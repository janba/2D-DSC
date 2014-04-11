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
    DeformableSimplicialComplex::DeformableSimplicialComplex(real AVG_EDGE_LENGTH, const std::vector<real>& points, const std::vector<int>& faces, DesignDomain *domain): design_domain(domain)
    {
        set_avg_edge_length(AVG_EDGE_LENGTH);
        MIN_ANGLE = M_PI * 10./180.;
        COS_MIN_ANGLE = cos(MIN_ANGLE);
        DEG_ANGLE = 0.2*MIN_ANGLE;
        
        MAX_LENGTH = 2.;
        MIN_LENGTH = 0.5;
        DEG_LENGTH = 0.2*MIN_LENGTH;
        
        MAX_AREA = 5.;
        MIN_AREA = 0.2;
        DEG_AREA = 0.2*MIN_AREA;
        
        INTERFACE_COLOR = DARK_RED;
        CROSSING_COLOR = RED;
        OUTSIDE_COLOR = BLACK;
        DEFAULT_COLOR = DARK_BLUE;
        OUTSIDE_FACE_COLOR = INVISIBLE;
        DEFAULT_FACE_COLOR = BLUE;
        
        create_simplicial_complex(points, faces);
        
        destination = HMesh::VertexAttributeVector<vec2>(get_no_vertices(), vec2(0.));
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
        destination.cleanup(cleanup_map.vmap);
    }
    
    
    HMesh::VertexAttributeVector<vec3> DeformableSimplicialComplex::get_vertex_colors() const
    {
        HMesh::VertexAttributeVector<vec3> colors = HMesh::VertexAttributeVector<vec3>();
        for(auto vi = vertices_begin(); vi != vertices_end(); ++vi)
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
        for(auto hei = halfedges_begin(); hei != halfedges_end(); ++hei)
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
        for(auto fi = faces_begin(); fi != faces_end(); ++fi)
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
            for (auto hew = walker(*fi); !hew.full_circle(); hew = hew.circulate_face_cw())
            {
                i++;
            }
            assert(i == 3);
            assert(area(*fi) > EPSILON);
        }
    }
    
    void DeformableSimplicialComplex::fix_complex()
    {
        smooth();
        
        max_min_angle();
        
        remove_degenerate_edges();
        remove_degenerate_faces();
        
        smooth();
    }
    
    void DeformableSimplicialComplex::resize_complex()
    {
        thickening_interface();
        
        thinning_interface();
        
        thickening();
        
        thinning();
        
        fix_complex();
    }
    
    real DeformableSimplicialComplex::intersection_with_link(const node_key& vid, vec2 destination) const
    {
        vec2 pos = get_pos(vid);
        vec2 ray = destination - pos;
        
        real min_t = INFINITY;
        for(auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            real t = Util::intersection_ray_line(pos, ray, get_pos(hew.vertex()), get_pos(hew.next().vertex()));
            if(0. <= t && t < INFINITY)
            {
                min_t = Util::min(t, min_t);
            }
        }
#ifdef DEBUG
        assert(min_t < INFINITY);
#endif
        return min_t;
    }
    
    bool DeformableSimplicialComplex::move_vertex(node_key vid)
    {
        vec2 pos = get_pos(vid);
        vec2 destination = get_destination(vid);
        real l = Util::length(destination - pos);
        if (l < 1e-4 * AVG_LENGTH)
        {
            return true;
        }
        
        real max_l = l*intersection_with_link(vid, destination) - 1e-4 * AVG_LENGTH;
        l = Util::max(Util::min(0.5*max_l, l), 0.);
        set_pos(vid, pos + l*Util::normalize(destination - pos));
        
        if(Util::length(destination - get_pos(vid)) < 1e-4 * AVG_LENGTH)
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
            for (auto vi = vertices_begin(); vi != vertices_end(); vi++)
            {
                if(is_movable(*vi))
                {
                    work = work | !move_vertex(*vi);
                }
            }
            
            fix_complex();
            count++;
        }
        
        resize_complex();
        
        HMesh::IDRemap cleanup_map;
        cleanup_attributes(cleanup_map);
        
        init_attributes();
        update_attributes();
    }
    
    bool DeformableSimplicialComplex::is_movable(node_key vid) const
    {
        //return unsafe_editable(vid) && is_interface(vid);
        return unsafe_editable(vid) && (is_interface(vid) || is_crossing(vid));
    }
    
    bool DeformableSimplicialComplex::is_movable(edge_key eid) const
    {
        return unsafe_editable(eid) && is_interface(eid);
    }
    
    bool DeformableSimplicialComplex::unsafe_editable(node_key vid) const
    {
        return !boundary(*mesh, vid) && !is_crossing(vid);
    }
    
    bool DeformableSimplicialComplex::safe_editable(node_key vid) const
    {
        return unsafe_editable(vid) && !is_interface(vid);
    }
    
    bool DeformableSimplicialComplex::unsafe_editable(edge_key eid) const
    {
        return !boundary(*mesh, eid);
    }
    
    bool DeformableSimplicialComplex::safe_editable(edge_key eid) const
    {
        return unsafe_editable(eid) && !is_interface(eid);
    }
    
    real DeformableSimplicialComplex::max_move_distance() const
    {
        real dist, max_dist = 0.;
        for(auto vi = vertices_begin(); vi != vertices_end(); ++vi)
        {
            if(is_movable(*vi))
            {
                dist = (destination[*vi] - get_pos(*vi)).length();
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
                auto hew = walker(*eit);
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
    
    vec2 DeformableSimplicialComplex::get_pos(node_key vid) const
    {
        return vec2(mesh->pos(vid)[0], mesh->pos(vid)[1]);
    }
    
    std::vector<vec2> DeformableSimplicialComplex::get_pos(face_key fid) const
    {
        std::vector<vec2> positions;
        for (auto hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            positions.push_back( get_pos(hew.vertex()) );
        }
        return positions;
    }
    
    vec2 DeformableSimplicialComplex::get_destination(node_key vid) const
    {
        return vec2(destination[vid]);
    }
    
    std::vector<vec2> DeformableSimplicialComplex::get_destination(face_key fid) const
    {
        std::vector<vec2> positions;
        for (auto hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            positions.push_back(get_destination(hew.vertex()));
        }
        return positions;
    }
    
    std::vector<HMesh::VertexID> DeformableSimplicialComplex::get_verts(node_key vid, bool interface) const
    {
        std::vector<node_key> vids;
        for (auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if(!interface || is_interface(hew.halfedge()))
            {
                vids.push_back(hew.vertex());
            }
        }
        return vids;
    }
    
    std::vector<HMesh::VertexID> DeformableSimplicialComplex::get_verts(face_key fid) const
    {
        std::vector<node_key> vids;
        for (auto hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            vids.push_back(hew.vertex());
        }
        return vids;
    }
    
    std::vector<HMesh::HalfEdgeID> DeformableSimplicialComplex::get_edges(face_key fid) const
    {
        std::vector<edge_key> edges;
        for (auto hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            edges.push_back(hew.halfedge());
        }
        return edges;
    }
    
    std::vector<int> DeformableSimplicialComplex::get_interface_labels(node_key vid) const
    {
        std::vector<int> labels(2,NO_LABEL);
        if(is_interface(vid))
        {
            for (auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
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
    
    
    void DeformableSimplicialComplex::set_pos(node_key vid, vec2 p)
    {
        mesh->pos(vid) = CGLA::Vec3d(p[0], p[1], 0.f);
    }
    
    void DeformableSimplicialComplex::set_destination(const node_key& vid, const vec2& dest)
    {
        if(is_movable(vid))
        {
            vec2 vec = dest - get_pos(vid);
            clamp_vector(vid, vec);
            destination[vid] = get_pos(vid) + vec;
        }
    }
    
    void DeformableSimplicialComplex::init_attributes()
    {
        for(auto fi = faces_begin(); fi != faces_end(); ++fi)
        {
            init_attributes(*fi);
        }
        
        for(auto ei = halfedges_begin(); ei != halfedges_end(); ++ei)
        {
            init_attributes(*ei);
        }
        
        for(auto vi = vertices_begin(); vi != vertices_end(); ++vi)
        {
            init_attributes(*vi);
        }
    }
    
    void DeformableSimplicialComplex::init_attributes(node_key vid)
    {
        destination[vid] = get_pos(vid);
    }
    
    void DeformableSimplicialComplex::init_attributes(face_key fid, int label)
    {
        if(label != NO_LABEL)
        {
            face_labels[fid] = label;
        }
    }
    
    
    void DeformableSimplicialComplex::update_attributes(node_key vid, int label)
    {
        if(label != NO_LABEL)
        {
            vertex_labels[vid] = label;
            return;
        }
        
        int inter = 0;
        for(auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
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
    
    
    void DeformableSimplicialComplex::update_attributes(edge_key heid, int fa1, int fa2)
    {
        auto hew = walker(heid);
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
    
    
    void DeformableSimplicialComplex::update_attributes(face_key fid, int label)
    {
        if(label != NO_LABEL)
        {
            face_labels[fid] = label;
        }
        
        for(auto hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
        {
            update_attributes(hew.halfedge());
        }
    }
    
    void DeformableSimplicialComplex::update_locally(node_key vid)
    {
        for(auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if(!boundary(*mesh, hew.halfedge()))
            {
                update_attributes(hew.face());
            }
        }
    }
    
    void DeformableSimplicialComplex::update_attributes()
    {
        for(auto fi = faces_begin(); fi != faces_end(); ++fi)
        {
            update_attributes(*fi);
        }
    }
    
    
    bool DeformableSimplicialComplex::split(edge_key eid)
    {
        if (!unsafe_editable(eid)) {
            return false;
        }
        
        auto hew = walker(eid);
        face_key f1 = hew.face();
        node_key v1 = hew.next().vertex();
        face_key f2 = hew.opp().face();
        node_key v2 = hew.opp().next().vertex();
        
        // Split
        node_key vid = mesh->split_edge(eid);
        face_key newf1 = mesh->split_face_by_edge(f1, vid, v1);
        face_key newf2 = mesh->split_face_by_edge(f2, vid, v2);
        
        // Update attributes
        init_attributes(vid);
        for(auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
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
    
    bool DeformableSimplicialComplex::split(face_key fid)
    {
        int fa = get_label(fid);
        node_key vid = mesh->split_face_by_vertex(fid);
        
        init_attributes(vid);
        
        for(auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            init_attributes(hew.halfedge());
            init_attributes(hew.face(), fa);
        }
        
        update_locally(vid);
        return true;
    }
    
    real DeformableSimplicialComplex::min_quality(const std::vector<edge_key>& eids, const vec2& pos_old, const vec2& pos_new)
    {
        real min_q = INFINITY;
        for (auto e : eids)
        {
            auto hew = walker(e);
            if(Util::sign(Util::signed_area(get_pos(hew.vertex()), get_pos(hew.opp().vertex()), pos_old)) !=
               Util::sign(Util::signed_area(get_pos(hew.vertex()), get_pos(hew.opp().vertex()), pos_new)))
            {
                return -INFINITY;
            }
            min_q = Util::min(min_q, std::abs(Util::min_angle(get_pos(hew.vertex()), get_pos(hew.opp().vertex()), pos_new)));
        }
        return min_q;
    }
    
    bool DeformableSimplicialComplex::is_collapsable(HMesh::Walker hew, bool safe)
    {
        if(safe)
        {
            if(safe_editable(hew.opp().vertex()))
            {
                return true;
            }
        }
        else {
            if(unsafe_editable(hew.opp().vertex()))
            {
                return true;
            }
        }
        if(is_interface(hew.halfedge()))
        {
            vec2 p0 = get_pos(hew.opp().vertex());
            vec2 p1 = get_pos(hew.vertex());
            vec2 p2 = get_pos(previous_interface(hew).opp().vertex());
            return Util::cos_angle(p2, p0, p1) < -COS_MIN_ANGLE;
        }
        if(boundary(*mesh, hew.halfedge()))
        {
            vec2 p0 = get_pos(hew.opp().vertex());
            vec2 p1 = get_pos(hew.vertex());
            auto hw = hew.circulate_vertex_ccw();
            while (!boundary(*mesh, hw.halfedge()))
            {
                hw = hw.circulate_vertex_ccw();
            }
            vec2 p2 = get_pos(hw.vertex());
            return Util::cos_angle(p2, p0, p1) < EPSILON-1.;
        }
        return false;
    }
    
    bool DeformableSimplicialComplex::collapse(const edge_key& eid, bool safe)
    {
        if (!precond_collapse_edge(*mesh, eid) || !unsafe_editable(eid))
        {
            return false;
        }
        
        auto hew = walker(eid);
        bool n0_is_editable = is_collapsable(hew, safe);
        bool n1_is_editable = is_collapsable(hew.opp(), safe);
        
        if (!n0_is_editable && !n1_is_editable)
        {
            return false;
        }
        std::vector<real> test_weights;
        if (!n0_is_editable || !n1_is_editable)
        {
            test_weights = {0.};
            if(!n0_is_editable)
            {
                hew = hew.opp();
            }
        }
        else {
            test_weights = {0., 0.5, 1.};
        }
        
        std::vector<node_key> nids = {hew.opp().vertex(), hew.vertex()};
        std::vector<face_key> e_fids = {hew.face(), hew.opp().face()};
        std::vector<edge_key> eids0, eids1;
        
        for(auto hw = walker(hew.halfedge()); !hw.full_circle(); hw = hw.circulate_vertex_cw())
        {
            if(hw.face() != e_fids[0] && hw.face() != e_fids[1])
            {
                eids0.push_back(hw.next().halfedge());
            }
        }
        for(auto hw = walker(hew.opp().halfedge()); !hw.full_circle(); hw = hw.circulate_vertex_cw())
        {
            if(hw.face() != e_fids[0] && hw.face() != e_fids[1])
            {
                eids1.push_back(hw.next().halfedge());
            }
        }
        
        real q_max = -INFINITY;
        real weight;
        for (real w : test_weights)
        {
            vec2 p = (1.-w) * get_pos(nids[1]) + w * get_pos(nids[0]);
            real q = Util::min(min_quality(eids0, get_pos(nids[0]), p), min_quality(eids1, get_pos(nids[1]), p));
            
            if (q > q_max && ((!is_interface(nids[0]) && !is_interface(nids[1])) || design_domain->is_inside(p)))
            {
                q_max = q;
                weight = w;
            }
        }
        
        if(q_max > EPSILON)
        {
            if(!safe || q_max > MIN_ANGLE + EPSILON)
            {
                collapse(hew, weight);
                return true;
            }
        }
        return false;
    }
    
    bool DeformableSimplicialComplex::collapse(HMesh::Walker hew, real weight)
    {
        node_key vid = hew.vertex();
        vec2 p = (1.-weight) * get_pos(hew.vertex()) + weight * get_pos(hew.opp().vertex());
        vec2 d = (1.-weight) * get_destination(hew.vertex()) + weight * get_destination(hew.opp().vertex());
        
        update_attributes(hew.halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.prev().halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.opp().next().halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.next().halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.opp().prev().halfedge(), OUTSIDE, OUTSIDE);
        update_attributes(hew.opp().vertex(), OUTSIDE);
        
        mesh->collapse_edge(hew.halfedge());
        update_locally(vid);
        set_pos(vid, p);
        set_destination(vid, d);
        return true;
    }
    
    bool DeformableSimplicialComplex::collapse(const face_key& fid, bool safe)
    {
        for (auto e : sorted_face_edges(fid)) {
            if(collapse(e, safe))
            {
                return true;
            }
        }
        return false;
    }
    
    std::vector<HMesh::HalfEdgeID> DeformableSimplicialComplex::sorted_face_edges(face_key fid)
    {
        auto hew = walker(fid);
        std::vector<edge_key> edges(3);
        edges[0] = hew.halfedge();
        edges[1] = hew.next().halfedge();
        edges[2] = hew.prev().halfedge();
        if(length(edges[1]) < length(edges[0])) std::swap(edges[1],edges[0]);
        if(length(edges[2]) < length(edges[1])) std::swap(edges[2],edges[1]);
        if(length(edges[1]) < length(edges[0])) std::swap(edges[1],edges[0]);
        
        return edges;
    }
    
    real DeformableSimplicialComplex::length(edge_key eid) const
    {
        return HMesh::length(*mesh, eid);
    }
    
    real DeformableSimplicialComplex::length_new(edge_key eid) const
    {
        auto hew = walker(eid);
        return Util::length(get_destination(hew.vertex()) - get_destination(hew.opp().vertex()));
    }
    
    real DeformableSimplicialComplex::min_edge_length(face_key fid)
    {
        std::vector<vec2> p = get_pos(fid);
        return Util::min(Util::min(Util::length(p[0] - p[1]), Util::length(p[1] - p[2])), Util::length(p[0] - p[2]));
    }
    
    real DeformableSimplicialComplex::min_angle(face_key fid)
    {
        std::vector<vec2> p = get_pos(fid);
        return Util::min_angle(p[0], p[1], p[2]);
    }
    
    void DeformableSimplicialComplex::remove_needles()
    {
        std::vector<face_key> fvec(0);
        for(auto fi = faces_begin(); fi != faces_end(); ++fi)
        {
            fvec.push_back(*fi);
        }
        
        for(auto &f : fvec)
        {
            auto edges = sorted_face_edges(f);
            if(min_angle(f) < MIN_ANGLE
               && std::abs(length(edges[1]) - length(edges[0])) > std::abs(length(edges[2]) - length(edges[1]))
               && area(f) > DEG_AREA*AVG_AREA)
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
    
    void DeformableSimplicialComplex::remove_degenerate_edges()
    {
        for(auto ei = halfedges_begin(); ei != halfedges_end(); ei++)
        {
            if(mesh->in_use(*ei))
            {
                if(length(*ei) < DEG_LENGTH*AVG_LENGTH && !collapse(*ei, true))
                {
                    collapse(*ei, false);
                }
            }
        }
    }
    
    void DeformableSimplicialComplex::remove_degenerate_faces()
    {
        for(auto fi = faces_begin(); fi != faces_end(); ++fi)
        {
            if(mesh->in_use(*fi))
            {
                if((min_angle(*fi) < DEG_ANGLE || area(*fi) < DEG_AREA*AVG_AREA) && !collapse(*fi, true))
                {
                    collapse(*fi, false);
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
            for(auto hei = halfedges_begin(); hei != halfedges_end(); ++hei)
            {
                if(safe_editable(*hei) && precond_flip_edge(*mesh, *hei))
                {
                    auto hew = walker(*hei);
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
    
    vec2 DeformableSimplicialComplex::get_barycenter(node_key vid, bool interface) const
    {
        if(interface && !is_interface(vid))
        {
            return get_pos(vid);
        }
        
        vec2 avg_pos(0.);
        int n = 0;
        
        for (auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
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
    
    vec2 DeformableSimplicialComplex::filter_vertex(node_key vid, std::vector<real> &filter) const
    {
        if(!is_interface(vid)&&!is_crossing(vid))
        {
            return vec2(0.f);
        }
        vec2 sum_pos(0.);
        int n = 0;
        for (auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            if (is_interface(hew.halfedge()))
            {
                n++;
                auto forth = hew;
                auto back = hew.opp();
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
	
    
    vec2 DeformableSimplicialComplex::get_normal(node_key vid) const
    {
        if(!is_interface(vid))
        {
            return vec2(0.);
        }
        
        vec2 n(0.f), r(0.f);
        int i = 0;
        for(auto hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
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
    
    void DeformableSimplicialComplex::clamp_vector(const node_key& vid, vec2& vec) const
    {
        design_domain->clamp_vector(get_pos(vid), vec);
    }
    
    real DeformableSimplicialComplex::area(face_key fid) const
    {
        return HMesh::area(*mesh, fid);
    }
    
    real DeformableSimplicialComplex::area_destination(face_key fid) const
    {
        std::vector<vec2> positions = get_destination(fid);
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
        for(auto vi = vertices_begin(); vi != vertices_end(); ++vi,++i)
        {
            if(safe_editable(*vi))
            {
                positions[i] =  t * (get_barycenter(*vi, false) - get_pos(*vi)) + get_pos(*vi);
            }
        }
        i = 0;
        for(auto vi = vertices_begin(); vi != vertices_end(); ++vi,++i)
        {
            if(safe_editable(*vi))
            {
                set_pos(*vi, positions[i]);
            }
        }
    }
    
    
    void DeformableSimplicialComplex::thickening_interface()
    {
        if(MAX_LENGTH == INFINITY)
        {
            return;
        }
        
        std::vector<edge_key> edges;
        for(auto hei = halfedges_begin(); hei != halfedges_end(); ++hei)
        {
            auto hew = walker(*hei);
            if(is_movable(*hei) && get_label(hew.face()) < get_label(hew.opp().face()))
            {
                edges.push_back(*hei);
            }
        }
        
        for (auto e : edges)
        {
            auto hew = walker(e);
            if(length(e) > MAX_LENGTH*AVG_LENGTH || is_crossing(hew.vertex()) || is_crossing(hew.opp().vertex()))
            {
                bool success = split(e);
#ifdef DEBUG
                if(success)
                {
                    std::cout << "Split interface" << std::endl;
                }
#endif
            }
        }
    }
    
    
    void DeformableSimplicialComplex::thinning_interface()
    {
        if(MIN_LENGTH <= 0.)
        {
            return;
        }
        
        for(auto heit = halfedges_begin(); heit != halfedges_end(); heit++)
        {
            if(mesh->in_use(*heit))
            {
                if(is_movable(*heit) && length(*heit) < MIN_LENGTH*AVG_LENGTH)
                {
                    bool success = collapse(*heit, false);
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
    
    
    void DeformableSimplicialComplex::thickening()
    {
        if(MAX_AREA == INFINITY)
        {
            return;
        }
        
        std::vector<face_key> faces;
        for(auto fi = faces_begin(); fi != faces_end(); ++fi)
        {
            if(area(*fi) > MAX_AREA*AVG_AREA)
            {
                faces.push_back(*fi);
            }
        }
        
        for (auto f : faces)
        {
            if(area(f) > MAX_AREA*AVG_AREA)
            {
                bool success = split(sorted_face_edges(f).back());
#ifdef DEBUG
                if(success)
                {
                    std::cout << "Thickening" << std::endl;
                }
#endif
            }
        }
    }
    
    void DeformableSimplicialComplex::thinning()
    {
        if(MIN_AREA <= 0.)
        {
            return;
        }
        
        for(auto fi = faces_begin(); fi != faces_end(); fi++)
        {
            if(mesh->in_use(*fi) && area(*fi) < MIN_AREA*AVG_AREA)
            {
                bool success = collapse(*fi, true);
#ifdef DEBUG
                if(success)
                {
                    std::cout << "Thinning" << std::endl;
                }
#endif
            }
        }
    }
    
    
    bool operator<(const DeformableSimplicialComplex::PQElem& e0, const DeformableSimplicialComplex::PQElem& e1)
    {
        return e0.pri > e1.pri;
    }
    
    
    void DeformableSimplicialComplex::add_to_queue(HMesh::HalfEdgeAttributeVector<int>& touched, std::priority_queue<PQElem>& Q, edge_key h, const HMesh::EnergyFun& efun)
    {
        auto w = walker(h);
        edge_key ho = w.opp().halfedge();
        
        real energy = efun.delta_energy(*mesh, h);
        int t = touched[h] + 1;
        touched[h] = t;
        touched[ho] = t;
        if((energy<0) && (t < 10000)){
            Q.push(PQElem(energy, h, t));
        }
        
    }
    
    void DeformableSimplicialComplex::add_one_ring_to_queue(HMesh::HalfEdgeAttributeVector<int>& touched, std::priority_queue<PQElem>& Q, node_key v, const HMesh::EnergyFun& efun)
    {
        for(auto hew = walker(v); !hew.full_circle(); hew = hew.circulate_vertex_cw())
        {
            add_to_queue(touched, Q, hew.halfedge(), efun);
        }
    }
    
    void DeformableSimplicialComplex::priority_queue_optimization(const HMesh::EnergyFun& efun)
    {
        HMesh::HalfEdgeAttributeVector<int> touched(get_no_halfedges(), 0);
        std::priority_queue<PQElem> Q;
        
        for(auto h = halfedges_begin(); h != halfedges_end(); ++h){
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
                
                auto w = walker(elem.h);
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