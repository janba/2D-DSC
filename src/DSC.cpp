//
//  Deformabel Simplicial Complex (DSC) method
//  Copyright (C) 2013  Technical University of Denmark
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  See licence.txt for a copy of the GNU General Public License.

#include "DSC.h"
#include "draw.h"

namespace DSC2D
{    
    DeformableSimplicialComplex::DeformableSimplicialComplex(real AVG_EDGE_LENGTH_, const std::vector<real>& points, const std::vector<int>& faces, DesignDomain *domain): AVG_EDGE_LENGTH(AVG_EDGE_LENGTH_), design_domain(domain)
    {
        MIN_ANGLE = M_PI * 2./180.;
        COS_MIN_ANGLE = cos(MIN_ANGLE);
        DEG_ANGLE = 0.5*MIN_ANGLE;
        
        MAX_EDGE_LENGTH = 1.5*AVG_EDGE_LENGTH;
        MIN_EDGE_LENGTH = 0.5*AVG_EDGE_LENGTH;
        DEG_EDGE_LENGTH = 0.5*MIN_EDGE_LENGTH;
        
        MIN_DEFORMATION = DEG_EDGE_LENGTH;
        
        real avg_area = 0.5*std::sqrt(3./4.)*AVG_EDGE_LENGTH*AVG_EDGE_LENGTH;
        MAX_AREA = 1.5*avg_area;
        MIN_AREA = 0.5*avg_area;
        DEG_AREA = 0.5*MIN_AREA;
        
        INTERFACE_COLOR = DARK_RED;
        CROSSING_COLOR = RED;
        OUTSIDE_COLOR = BLACK;
        DEFAULT_COLOR = DARK_BLUE;
        OUTSIDE_FACE_COLOR = INVISIBLE;
        DEFAULT_FACE_COLOR = BLUE;
        
        create_simplicial_complex(points, faces);
        
        new_pos = HMesh::VertexAttributeVector<vec2>(get_no_vertices(), vec2(0.));
        vertex_labels = HMesh::VertexAttributeVector<int>(get_no_vertices(), OUTSIDE);
        edge_labels = HMesh::HalfEdgeAttributeVector<int>(get_no_halfedges(), OUTSIDE);
        face_labels = HMesh::FaceAttributeVector<int>(get_no_faces(), OUTSIDE);
    }
    
    void DeformableSimplicialComplex::cleanup_attributes(HMesh::IDRemap& cleanup_map)
    {
        mesh->cleanup(cleanup_map);
        vertex_labels.cleanup(cleanup_map.vmap);
        edge_labels.cleanup(cleanup_map.hmap);
        face_labels.cleanup(cleanup_map.fmap);
        new_pos.cleanup(cleanup_map.vmap);
    }
    
    
    HMesh::VertexAttributeVector<vec3> DeformableSimplicialComplex::get_vertex_colors() const
    {
        HMesh::VertexAttributeVector<vec3> colors = HMesh::VertexAttributeVector<vec3>();
        for(HMesh::VertexIDIterator vi = vertices_begin(); vi != vertices_end(); ++vi)
        {
            switch (vertex_labels[*vi]) {
                case INTERFACE:
                    colors[*vi] = INTERFACE_COLOR;
                    break;
                case CROSSING:
                    colors[*vi] = CROSSING_COLOR;
                    break;
                case OUTSIDE:
                    colors[*vi] = OUTSIDE_COLOR;
                    break;
                default:
                    colors[*vi] = DEFAULT_COLOR;
                    break;
            }
        }
        return colors;
    }
    
    HMesh::HalfEdgeAttributeVector<vec3> DeformableSimplicialComplex::get_edge_colors() const
    {
        HMesh::HalfEdgeAttributeVector<vec3> colors = HMesh::HalfEdgeAttributeVector<vec3>();
        for(HMesh::HalfEdgeIDIterator hei = halfedges_begin(); hei != halfedges_end(); ++hei)
        {
            switch (edge_labels[*hei]) {
                case INTERFACE:
                    colors[*hei] = INTERFACE_COLOR;
                    break;
                case OUTSIDE:
                    colors[*hei] = OUTSIDE_COLOR;
                    break;
                default:
                    colors[*hei] = DEFAULT_COLOR;
                    break;
            }
        }
        return colors;
    }
    
    HMesh::FaceAttributeVector<vec3> DeformableSimplicialComplex::get_face_colors() const
    {
        HMesh::FaceAttributeVector<vec3> colors = HMesh::FaceAttributeVector<vec3>();
        for(HMesh::FaceIDIterator fi = faces_begin(); fi != faces_end(); ++fi)
        {
            switch (get_label(*fi)) {
                case OUTSIDE:
                    colors[*fi] = OUTSIDE_FACE_COLOR;
                    break;
                default:
                    colors[*fi] = Util::color(DEFAULT_FACE_COLOR, get_label(*fi));
                    break;
            }
        }
        return colors;
    }
    
    void DeformableSimplicialComplex::create_simplicial_complex(const std::vector<real>& points, const std::vector<int>& faces)
    {
        mesh = new HMesh::Manifold();
        std::vector<int> temp(faces.size()/3,3);
        mesh->build(points.size()/3, &points[0], temp.size(), &temp[0], &faces[0]);
    }
    
    void DeformableSimplicialComplex::validity_check()
    {
        for (auto fi = faces_begin(); fi != faces_end(); fi++) {
            int i = 0;
            for (HMesh::Walker hew = walker(*fi); !hew.full_circle(); hew = hew.circulate_face_cw())
            {
                i++;
            }
            assert(i == 3);
            assert(area(*fi) > EPSILON);
        }
    }
    
    void DeformableSimplicialComplex::fix_mesh()
    {
        smooth();
        
        remove_needles();
        
        max_min_angle();
        
        remove_caps();
        
        remove_degenerate_faces();
        
        smooth();
    }
    
    void DeformableSimplicialComplex::resize_elements()
    {
        bool work = true;
        int count = 0;
        while (work && count < 5)
        {
            work = split_interface();
            
            work = collapse_interface() | work;
            
            work = thickening() | work;
            
            work = thinning() | work;
            
            fix_mesh();
            count++;
        }
    }
    
    real DeformableSimplicialComplex::intersection_with_link(const HMesh::VertexID& vid, vec2 destination) const
    {
        real scale = INFINITY;
        vec2 p = get_pos(vid);
        vec2 r = destination - p;
        vec2 q, s;
        for(HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            q = get_pos(hew.vertex());
            s = get_pos(hew.next().vertex()) - q;
            real t = Util::intersection(p, r, q, s);
            if(0. <= t && t < INFINITY)
            {
                scale = t;
            }
        }
#ifdef DEBUG
        assert(scale < INFINITY);
#endif
        if (scale == INFINITY) {
            scale = 0.;
        }
        return scale;
    }
    
    bool DeformableSimplicialComplex::move_vertex(HMesh::VertexID vid)
    {
        vec2 v0_new = get_pos_new(vid);
        vec2 v0 = get_pos(vid);
        real l = (v0 - v0_new).length();
        if (l < EPSILON)
        {
            return true;
        }
        
        real scale = intersection_with_link(vid, v0_new);
        l = Util::max(Util::min(l*scale - MIN_DEFORMATION, l), 0.);
        vec2 v0_temp = v0 + l*normalize(v0_new - v0);
        
        vec2 v1, v2;
        for(HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            v1 = get_pos(hew.vertex());
            v2 = get_pos(hew.next().vertex());
            
            if (!Util::is_left_of(v0_temp, v1, v2)) {
                return false;
            }
        }
        
        set_pos(vid, v0_temp);
        
        if((v0_temp - v0_new).length() < EPSILON)
        {
            return true;
        }
        return false;
    }
    
    void DeformableSimplicialComplex::deform()
    {
        bool work = true;
        int count = 0;
        while(work && count < 10)
        {
            work = false;
            for (HMesh::VertexIDIterator vi = vertices_begin(); vi != vertices_end(); vi++)
            {
                if(is_movable(*vi))
                {
                    work = work | !move_vertex(*vi);
                }
            }
            
            fix_mesh();
            count++;
        }
        
        resize_elements();
        
        HMesh::IDRemap cleanup_map;
        cleanup_attributes(cleanup_map);
        
        init_attributes();
        update_attributes();
    }
    
    bool DeformableSimplicialComplex::is_movable(HMesh::VertexID vid) const
    {
        //return unsafe_editable(vid) && is_interface(vid);
        return unsafe_editable(vid) && (is_interface(vid) || is_crossing(vid));
    }
    
    bool DeformableSimplicialComplex::is_movable(HMesh::HalfEdgeID eid) const
    {
        return unsafe_editable(eid) && is_interface(eid);
    }
    
    bool DeformableSimplicialComplex::unsafe_editable(HMesh::VertexID vid) const
    {
        return !boundary(*mesh, vid);
    }
    
    bool DeformableSimplicialComplex::safe_editable(HMesh::VertexID vid) const
    {
        return unsafe_editable(vid) && !is_interface(vid) && !is_crossing(vid);
    }
    
    bool DeformableSimplicialComplex::unsafe_editable(HMesh::HalfEdgeID eid) const
    {
        return !boundary(*mesh, eid);
    }
    
    bool DeformableSimplicialComplex::safe_editable(HMesh::HalfEdgeID eid) const
    {
        return unsafe_editable(eid) && !is_interface(eid);
    }
    
    real DeformableSimplicialComplex::max_move_distance() const
    {
        real dist, max_dist = 0.;
        for(HMesh::VertexIDIterator vi = vertices_begin(); vi != vertices_end(); ++vi)
        {
            if(is_movable(*vi))
            {
                dist = (new_pos[*vi] - get_pos(*vi)).length();
                max_dist = Util::max(max_dist, dist);
            }
        }
        return max_dist;
    }
    
    std::vector<vec2> DeformableSimplicialComplex::get_design_variable_positions()
    {
        std::vector<vec2> p;
        for (auto vi = vertices_begin(); vi != vertices_end(); vi++)
        {
            if(is_movable(*vi))
            {
                p.push_back(get_pos(*vi));
            }
        }
        return p;
    }
    
    std::vector<vec2> DeformableSimplicialComplex::get_interface_edge_positions()
    {
        std::vector<vec2> p;
        for (auto eit = halfedges_begin(); eit != halfedges_end(); eit++)
        {
            if(is_interface(*eit))
            {
                HMesh::Walker hew = walker(*eit);
                p.push_back(get_pos(hew.vertex()));
                p.push_back(get_pos(hew.opp().vertex()));
            }
        }
        return p;
    }
    
    vec2 DeformableSimplicialComplex::get_center()
    {
        return design_domain->get_center();
    }
    
    vec2 DeformableSimplicialComplex::get_pos(HMesh::VertexID vid) const
    {
        return vec2(mesh->pos(vid)[0], mesh->pos(vid)[1]);
    }
    
    std::vector<vec2> DeformableSimplicialComplex::get_pos(HMesh::FaceID fid) const
    {
        std::vector<vec2> positions;
        for (HMesh::Walker hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            positions.push_back( get_pos(hew.vertex()) );
        }
        return positions;
    }
    
    vec2 DeformableSimplicialComplex::get_pos_new(HMesh::VertexID vid) const
    {
        return vec2(new_pos[vid]);
    }
    
    std::vector<vec2> DeformableSimplicialComplex::get_pos_new(HMesh::FaceID fid) const
    {
        std::vector<vec2> positions;
        for (HMesh::Walker hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            positions.push_back(get_pos_new(hew.vertex()));
        }
        return positions;
    }
    
    std::vector<HMesh::VertexID> DeformableSimplicialComplex::get_verts(HMesh::VertexID vid, bool interface) const
    {
        std::vector<HMesh::VertexID> vids;
        for (HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if(!interface || is_interface(hew.halfedge()))
            {
                vids.push_back(hew.vertex());
            }
        }
        return vids;
    }
    
    std::vector<HMesh::VertexID> DeformableSimplicialComplex::get_verts(HMesh::FaceID fid) const
    {
        std::vector<HMesh::VertexID> vids;
        for (HMesh::Walker hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            vids.push_back(hew.vertex());
        }
        return vids;
    }
    
    std::vector<HMesh::HalfEdgeID> DeformableSimplicialComplex::get_edges(HMesh::FaceID fid) const
    {
        std::vector<HMesh::HalfEdgeID> edges;
        for (HMesh::Walker hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            edges.push_back(hew.halfedge());
        }
        return edges;
    }
    
    std::vector<int> DeformableSimplicialComplex::get_interface_labels(HMesh:: VertexID vid) const
    {
        std::vector<int> labels(2,NO_LABEL);
        if(is_interface(vid))
        {
            for (HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
            {
                if(is_interface(hew.halfedge()))
                {
                    labels[0] = Util::min(get_label(hew.face()),	get_label(hew.opp().face()));
                    labels[1] = Util::max(get_label(hew.face()),	get_label(hew.opp().face()));
                }
            }
        }
        return labels;
    }
    
    
    void DeformableSimplicialComplex::set_pos(HMesh::VertexID vid, vec2 p)
    {
        mesh->pos(vid) = CGLA::Vec3d(p[0], p[1], 0.f);
    }
    
    void DeformableSimplicialComplex::set_destination(HMesh::VertexID vid, vec2 dest)
    {
        if(is_movable(vid))
        {
            vec2 vec = dest - get_pos(vid);
            clamp_vector(vid, vec);
            new_pos[vid] = get_pos(vid) + vec;
        }
    }
    
    void DeformableSimplicialComplex::init_attributes()
    {
        for(HMesh::FaceIDIterator fi = faces_begin(); fi != faces_end(); ++fi)
        {
            init_attributes(*fi);
        }
        
        for(HMesh::HalfEdgeIDIterator ei = halfedges_begin(); ei != halfedges_end(); ++ei)
        {
            init_attributes(*ei);
        }
        
        for(HMesh::VertexIDIterator vi = vertices_begin(); vi != vertices_end(); ++vi)
        {
            init_attributes(*vi);
        }
    }
    
    void DeformableSimplicialComplex::init_attributes(HMesh::VertexID vid)
    {
        new_pos[vid] = get_pos(vid);
    }
    
    void DeformableSimplicialComplex::init_attributes(HMesh::FaceID fid, int label)
    {
        if(label != NO_LABEL)
        {
            face_labels[fid] = label;
        }
    }
    
    
    void DeformableSimplicialComplex::update_attributes(HMesh::VertexID vid, int label)
    {
        if(label != NO_LABEL)
        {
            vertex_labels[vid] = label;
            return;
        }
        
        int inter = 0;
        for(HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if(is_interface(hew.halfedge()))
            {
                inter++;
            }
        }
        if(inter > 2)
        {
            vertex_labels[vid] = CROSSING;
        }
        else if(inter > 0)
        {
            vertex_labels[vid] = INTERFACE;
        }
        else if(!boundary(*mesh, vid))
        {
            vertex_labels[vid] = get_label(walker(vid).face());
        }
        else
        {
            vertex_labels[vid] = OUTSIDE;
        }
    }
    
    
    void DeformableSimplicialComplex::update_attributes(HMesh::HalfEdgeID heid, int fa1, int fa2)
    {
        HMesh::Walker hew = walker(heid);
        if((fa1 == NO_LABEL || fa2 == NO_LABEL))
        {
            if(!boundary(*mesh, heid))
            {
                fa1 = get_label(hew.face());
                fa2 = get_label(hew.opp().face());
            }
            else {
                fa1 = OUTSIDE;
                fa2 = OUTSIDE;
            }
        }
        
        if (fa1 != fa2)
        {
            edge_labels[heid] = INTERFACE;
            edge_labels[hew.opp().halfedge()] = INTERFACE;
        }
        else
        {
            edge_labels[heid] = fa1;
            edge_labels[hew.opp().halfedge()] = fa1;
        }
        
        update_attributes(hew.vertex());
        update_attributes(hew.opp().vertex());
    }
    
    
    void DeformableSimplicialComplex::update_attributes(HMesh::FaceID fid, int label)
    {
        if(label != NO_LABEL)
        {
            face_labels[fid] = label;
        }
        
        for(HMesh::Walker hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            update_attributes(hew.halfedge());
        }
    }
    
    void DeformableSimplicialComplex::update_locally(HMesh::VertexID vid)
    {
        for(HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if(!boundary(*mesh, hew.halfedge()))
            {
                DeformableSimplicialComplex::update_attributes(hew.face());
            }
        }
    }
    
    void DeformableSimplicialComplex::update_attributes()
    {
        for(HMesh::FaceIDIterator fi = faces_begin(); fi != faces_end(); ++fi)
        {
            update_attributes(*fi);
        }
    }
    
    
    bool DeformableSimplicialComplex::split(HMesh::HalfEdgeID eid)
    {
        if (!unsafe_editable(eid)) {
            return false;
        }
        
        HMesh::Walker hew = walker(eid);
        HMesh::FaceID f1 = hew.face();
        HMesh::VertexID v1 = hew.next().vertex();
        HMesh::FaceID f2 = hew.opp().face();
        HMesh::VertexID v2 = hew.opp().next().vertex();
        
        // Split
        HMesh::VertexID vid = mesh->split_edge(eid);
        HMesh::FaceID newf1 = mesh->split_face_by_edge(f1, vid, v1);
        HMesh::FaceID newf2 = mesh->split_face_by_edge(f2, vid, v2);
        
        // Update attributes
        init_attributes(vid);
        for(HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if(hew.halfedge() != eid && hew.opp().halfedge() != eid)
            {
                init_attributes(hew.halfedge());
            }
        }
        init_attributes(newf1, get_label(f1));
        init_attributes(newf2, get_label(f2));
        
        update_locally(vid);
        return true;
    }
    
    bool DeformableSimplicialComplex::split(HMesh::FaceID fid)
    {
        int fa = get_label(fid);
        HMesh::VertexID vid = mesh->split_face_by_vertex(fid);
        
        init_attributes(vid);
        
        for(HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            init_attributes(hew.halfedge());
            init_attributes(hew.face(), fa);
        }
        
        update_locally(vid);
        return true;
    }
    
    
    bool DeformableSimplicialComplex::safe_collapse(HMesh::HalfEdgeID eid)
    {
        std::vector<vec2> positions;
        for(HMesh::Walker hew = walker(eid); !hew.full_circle(); hew = hew.circulate_vertex_ccw())
        {
            positions.push_back(get_pos(hew.vertex()));
        }
        
        real a, A;
        for(int j = 1; j < positions.size() - 1; j++)
        {
            A = Util::signed_area(positions[0], positions[j], positions[j+1]);
            a = Util::min_angle(positions[0], positions[j], positions[j+1]);
            if(a < MIN_ANGLE || A < EPSILON || A > MAX_AREA)
            {
                return false;
            }
        }
        return unsafe_collapse(eid);
    }
    
    bool DeformableSimplicialComplex::unsafe_collapse(HMesh::HalfEdgeID eid)
    {
        HMesh::Walker hew = walker(eid);
        if (!precond_collapse_edge(*mesh, eid) || !unsafe_editable(eid) || !unsafe_editable(hew.opp().vertex())) {
            return false;
        }
        
        HMesh::VertexID vid = hew.vertex();
        
        update_attributes(eid, OUTSIDE, OUTSIDE);
        update_attributes(hew.prev().halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.opp().next().halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.next().halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.opp().prev().halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.opp().vertex(), OUTSIDE);
        
        mesh->collapse_edge(eid);
        
        update_locally(vid);
        return true;
    }
    
    std::vector<HMesh::HalfEdgeID> DeformableSimplicialComplex::sorted_face_edges(HMesh::FaceID fid)
    {
        HMesh::Walker hew = walker(fid);
        std::vector<HMesh::HalfEdgeID> edges(3);
        edges[0] = hew.halfedge();
        edges[1] = hew.next().halfedge();
        edges[2] = hew.prev().halfedge();
        if(length(edges[1]) < length(edges[0])) std::swap(edges[1],edges[0]);
        if(length(edges[2]) < length(edges[1])) std::swap(edges[2],edges[1]);
        if(length(edges[1]) < length(edges[0])) std::swap(edges[1],edges[0]);
        
        return edges;
    }
    
    real DeformableSimplicialComplex::length(HMesh::HalfEdgeID eid) const
    {
        return HMesh::length(*mesh, eid);
    }
    
    real DeformableSimplicialComplex::length_new(HMesh::HalfEdgeID eid) const
    {
        HMesh::Walker hew = walker(eid);
        return Util::length(get_pos_new(hew.vertex()) - get_pos_new(hew.opp().vertex()));
    }
    
    real DeformableSimplicialComplex::min_edge_length(HMesh::FaceID fid)
    {
        std::vector<vec2> p = get_pos(fid);
        return Util::min(Util::min(Util::length(p[0] - p[1]), Util::length(p[1] - p[2])), Util::length(p[0] - p[2]));
    }
    
    real DeformableSimplicialComplex::min_angle(HMesh::FaceID fid)
    {
        std::vector<vec2> p = get_pos(fid);
        return Util::min_angle(p[0], p[1], p[2]);
    }
    
    void DeformableSimplicialComplex::remove_needles()
    {
        std::vector<HMesh::FaceID> fvec(0);
        for(HMesh::FaceIDIterator fi = faces_begin(); fi != faces_end(); ++fi)
        {
            fvec.push_back(*fi);
        }
        
        for(auto &f : fvec)
        {
            auto edges = sorted_face_edges(f);
            if(min_angle(f) < MIN_ANGLE
               && std::abs(length(edges[1]) - length(edges[0])) > std::abs(length(edges[2]) - length(edges[1]))
               && area(f) > DEG_AREA)
                //&& length(m,edges[2])/length(m,edges[1]) < 1.3
                //&& length(m,edges[2])/length(m,edges[1]) > 0.7)// length(m,edges[2])/length(m,edges[0]) > SPLIT_EDGE_RATIO)
            {
#ifdef DEBUG
                if (split(f))
                {
                    std::cout << "Split needle" << std::endl;
                }
#else
                split(f);
#endif
            }
        }
    }
    
    void DeformableSimplicialComplex::remove_degenerate_faces()
    {
        for(HMesh::FaceIDIterator fi = faces_begin(); fi != faces_end(); ++fi)
        {
            if(mesh->in_use(*fi))
            {
                if(min_edge_length(*fi) < DEG_EDGE_LENGTH ||
                   min_angle(*fi) < DEG_ANGLE || area(*fi) < DEG_AREA)
                {
                    HMesh::Walker hew = walker(sorted_face_edges(*fi)[0]);
                    vec2 next = get_pos(hew.vertex()) - get_pos(hew.next().vertex());
                    vec2 prev = get_pos(hew.prev().vertex()) - get_pos(hew.prev().prev().vertex());
                    if(sqr_length(next) < sqr_length(prev) && unsafe_editable(hew.vertex()))
                    {
                        hew = hew.opp();
                    }
#ifdef DEBUG
                    if (unsafe_collapse(hew.halfedge()))
                    {
                        std::cout << "Remove degenerate face" << std::endl;
                    }
#else
                    unsafe_collapse(hew.halfedge());
#endif
                }
            }
        }
    }
    
    void DeformableSimplicialComplex::remove_caps()
    {
        bool change = true;
        while (change)
        {
            change = false;
            vec2 p0, p1, p2, p3;
            int fa;
            for(HMesh::HalfEdgeIDIterator hei = halfedges_begin(); hei != halfedges_end(); ++hei)
            {
                if(unsafe_editable(*hei) && precond_flip_edge(*mesh, *hei))
                {
                    HMesh::Walker hew = walker(*hei);
                    p0 = get_pos(hew.vertex());
                    p1 = get_pos(hew.next().vertex());
                    p2 = get_pos(hew.opp().vertex());
                    p3 = get_pos(hew.opp().next().vertex());
                    
                    if(Util::cos_angle(p0,p1,p2) < -COS_MIN_ANGLE &&
                       Util::cos_angle(p1,p2,p3) > COS_MIN_ANGLE &&
                       Util::cos_angle(p3,p0,p1) > COS_MIN_ANGLE &&
                       Util::signed_area(p0,p1,p3) > 0. && Util::signed_area(p2,p3,p1) > 0.)
                    {
                        area(hew.face()) > area(hew.opp().face()) ? fa = get_label(hew.face()) : fa = get_label(hew.opp().face());
#ifdef DEBUG
                        std::cout << "Flip cap" << std::endl;
#endif
                        mesh->flip_edge(*hei);
                        change = true;
                        update_attributes(hew.face(), fa);
                        update_attributes(hew.opp().face(), fa);
                    }
                }
            }
        }
    }
    
    vec2 DeformableSimplicialComplex::get_barycenter(HMesh::VertexID vid, bool interface) const
    {
        if(interface && !is_interface(vid))
        {
            return get_pos(vid);
        }
        
        vec2 avg_pos(0.);
        int n = 0;
        
        for (HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if(is_interface(hew.halfedge()) || !interface)
            {
                avg_pos += get_pos(hew.vertex());
                n++;
            }
        }
#ifdef DEBUG
        assert(n > 0);
        assert(!Util::isnan(avg_pos[0]) && !Util::isnan(avg_pos[1]));
#endif
        return avg_pos/n;
    }
    
    HMesh::Walker DeformableSimplicialComplex::next_interface(HMesh::Walker hw) const
    {
        if (!is_interface(hw.halfedge())) // should only be used for interface halfedges
        {
            return hw; // return NULL;
        }
        hw = hw.next(); // to circulate around pointing-to vertex
        while (!is_interface(hw.halfedge()))
        {
            hw = hw.circulate_vertex_cw();
        }
        return hw;
    }
    
    HMesh::Walker DeformableSimplicialComplex::previous_interface(HMesh::Walker hw) const
    {
        if (!is_interface(hw.halfedge())) // should only be used for interface halfedges
        {
            return hw; // return NULL;
        }
        hw = hw.circulate_vertex_ccw(); // to move away from interface
        while (!is_interface(hw.halfedge()))
        {
            hw = hw.circulate_vertex_ccw();
        }
        return hw.opp();
    }
    
    vec2 DeformableSimplicialComplex::filter_vertex(HMesh::VertexID vid, std::vector<real> &filter) const
    {
        if(!is_interface(vid)&&!is_crossing(vid))
        {
            return vec2(0.f);
        }
        vec2 sum_pos(0.);
        int n = 0;
        for (HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if (is_interface(hew.halfedge()))
            {
                n++;
                HMesh::Walker forth = hew;
                HMesh::Walker back = hew.opp();
                for (int i= 1; i<filter.size(); i++, forth = next_interface(forth), back = previous_interface(back))
                {
                    sum_pos += filter[i]*get_pos(forth.vertex());
                    sum_pos += filter[i]*get_pos(back.opp().vertex());
                }
            }
        }
        sum_pos += 2*n*filter[0]*get_pos(vid);
        return sum_pos/(2*n);
    }
	
    
    vec2 DeformableSimplicialComplex::normal(HMesh::VertexID vid) const
    {
        if(!is_interface(vid))
        {
            return vec2(0.);
        }
        
        vec2 n(0.f), r(0.f);
        int i = 0;
        for(HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if(is_interface(hew.halfedge()))
            {
                if(get_label(hew.face()) > get_label(hew.opp().face()))
                {
                    r = normalize(get_pos(hew.vertex()) - get_pos(vid));
                }
                else
                {
                    r = normalize(get_pos(vid) - get_pos(hew.vertex()));
                }
                n += vec2(r[1], -r[0]);
                i++;
            }
        }
#ifdef DEBUG
        assert(i == 2);
        assert(!Util::isnan(n[0]) && !Util::isnan(n[1]));
#endif
        if(sqr_length(n) > EPSILON)
        {
            n.normalize();
        }
        return n;
    }
    
    void DeformableSimplicialComplex::clamp_vector(const HMesh::VertexID& vid, vec2& vec) const
    {
        design_domain->clamp_vector(get_pos(vid), vec);
    }
    
    real DeformableSimplicialComplex::area(HMesh::FaceID fid) const
    {
        return HMesh::area(*mesh, fid);
    }
    
    real DeformableSimplicialComplex::area_new(HMesh::FaceID fid) const
    {
        std::vector<vec2> positions = get_pos_new(fid);
        real A = std::abs(Util::signed_area(positions[0], positions[1], positions[2]));
#ifdef DEBUG
        assert(A > 0.);
#endif
        return A;
    }
    
    void DeformableSimplicialComplex::smooth(real t)
    {
        std::vector<vec2> positions(get_no_vertices());
        int i = 0;
        for(HMesh::VertexIDIterator vi = vertices_begin(); vi != vertices_end(); ++vi,++i)
        {
            if(safe_editable(*vi))
            {
                positions[i] =  t * (get_barycenter(*vi, false) - get_pos(*vi)) + get_pos(*vi);
            }
        }
        i = 0;
        for(HMesh::VertexIDIterator vi = vertices_begin(); vi != vertices_end(); ++vi,++i)
        {
            if(safe_editable(*vi))
            {
                set_pos(*vi, positions[i]);
            }
        }
    }
    
    
    bool DeformableSimplicialComplex::split_interface()
    {
        bool change = false;
        std::vector<HMesh::HalfEdgeID> half_edges;
        for(HMesh::HalfEdgeIDIterator hei = halfedges_begin(); hei != halfedges_end(); ++hei)
        {
            HMesh::Walker hew = walker(*hei);
            if(is_movable(*hei) && get_label(hew.face()) < get_label(hew.opp().face()))
            {
                half_edges.push_back(*hei);
            }
        }
        
        for (auto heit = half_edges.begin(); heit != half_edges.end(); heit++)
        {
            HMesh::Walker hew = walker(*heit);
            if(length(*heit) > MAX_EDGE_LENGTH || is_crossing(hew.vertex()) || is_crossing(hew.opp().vertex()))
            {
                bool success = split(*heit);
                change = success | change;
#ifdef DEBUG
                if(success)
                {
                    std::cout << "Split interface" << std::endl;
                }
#endif
            }
        }
        return change;
    }
    
    
    bool DeformableSimplicialComplex::collapse_interface()
    {
        bool change = false;
        for(HMesh::HalfEdgeIDIterator heit = halfedges_begin(); heit != halfedges_end(); heit++)
        {
            if(mesh->in_use(*heit))
            {
                if(is_movable(*heit) && length(*heit) < MIN_EDGE_LENGTH)
                {
                    HMesh::VertexID vid = walker(*heit).opp().vertex();
                    std::vector<HMesh::VertexID> vertices = get_verts(vid, true);
                    real angle = Util::cos_angle(get_pos(vertices[0]), get_pos(vid), get_pos(vertices[1]));
                    //                real angle = dot(normalize(get_pos(vertices[0]) - get_pos(vid)), normalize(get_pos(vertices[1]) - get_pos(vid)));
                    if(angle < -COS_MIN_ANGLE || length(*heit) < DEG_EDGE_LENGTH)
                    {
                        bool success = safe_collapse(*heit);
                        change = success | change;
                        
#ifdef DEBUG
                        if(success)
                        {
                            std::cout << "Collapse interface" << std::endl;
                        }
#endif
                    }
                }
            }
        }
        return change;
    }
    
    
    bool DeformableSimplicialComplex::thickening()
    {
        std::vector<HMesh::FaceID> faces;
        for(HMesh::FaceIDIterator fi = faces_begin(); fi != faces_end(); ++fi)
        {
            faces.push_back(*fi);
        }
        
        bool change = false;
        for (auto fi = faces.begin(); fi != faces.end(); fi++)
        {
            if(area(*fi) > MAX_AREA && !is_outside(*fi))
            {
                bool success = split(*fi);
                change = success | change;
#ifdef DEBUG
                if(success)
                {
                    std::cout << "Thickening" << std::endl;
                }
#endif
            }
        }
        return change;
    }
    
    bool DeformableSimplicialComplex::thinning()
    {
        bool change = false;
        for(HMesh::HalfEdgeIDIterator hei = halfedges_begin(); hei != halfedges_end(); ++hei)
        {
            if(mesh->in_use(*hei))
            {
                if(safe_editable(walker(*hei).opp().vertex()) && safe_editable(*hei))
                {
                    bool success = safe_collapse(*hei);
                    change = success | change;
#ifdef DEBUG
                    if(success)
                    {
                        std::cout << "Thinning" << std::endl;
                    }
#endif
                }
            }
        }
        return change;
    }
    
    
    bool operator<(const PQElem& e0, const PQElem& e1)
    {
        return e0.pri > e1.pri;
    }
    
    
    void DeformableSimplicialComplex::add_to_queue(HMesh::HalfEdgeAttributeVector<int>& touched, std::priority_queue<PQElem>& Q, HMesh::HalfEdgeID h, const HMesh::EnergyFun& efun)
    {
        HMesh::Walker w = walker(h);
        HMesh::HalfEdgeID ho = w.opp().halfedge();
        
        real energy = efun.delta_energy(*mesh, h);
        int t = touched[h] + 1;
        touched[h] = t;
        touched[ho] = t;
        if((energy<0) && (t < 10000)){
            Q.push(PQElem(energy, h, t));
        }
        
    }
    
    void DeformableSimplicialComplex::add_one_ring_to_queue(HMesh::HalfEdgeAttributeVector<int>& touched, std::priority_queue<PQElem>& Q, HMesh::VertexID v, const HMesh::EnergyFun& efun)
    {
        for(HMesh::Walker hew = walker(v); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            add_to_queue(touched, Q, hew.halfedge(), efun);
        }
    }
    
    void DeformableSimplicialComplex::priority_queue_optimization(const HMesh::EnergyFun& efun)
    {
        HMesh::HalfEdgeAttributeVector<int> touched(get_no_halfedges(), 0);
        std::priority_queue<PQElem> Q;
        
        for(HMesh::HalfEdgeIDIterator h = halfedges_begin(); h != halfedges_end(); ++h){
            if(!touched[*h])
            {
                add_to_queue(touched, Q, *h, efun);
            }
        }
        
        while(!Q.empty())
        {
            PQElem elem = Q.top();
            Q.pop();
            
            if(touched[elem.h] == elem.time && precond_flip_edge(*mesh, elem.h) && safe_editable(elem.h))
            {
                mesh->flip_edge(elem.h);
                
                HMesh::Walker w = walker(elem.h);
                add_one_ring_to_queue(touched, Q, w.vertex(), efun);
                add_one_ring_to_queue(touched, Q, w.next().vertex(), efun);
                add_one_ring_to_queue(touched, Q, w.opp().vertex(), efun);
                add_one_ring_to_queue(touched, Q, w.opp().next().vertex(), efun);
            }
        }
    }
    
    
    void DeformableSimplicialComplex::max_min_angle()
    {
        HMesh::MinAngleEnergy energy_fun(MIN_ANGLE);
        priority_queue_optimization(energy_fun);
    }
    
}