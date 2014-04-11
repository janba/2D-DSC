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

#pragma once

#include <queue>
#include "util.h"
#include "design_domain.h"

#ifdef WIN32
#include <HMesh/Manifold.h>
#include <HMesh/mesh_optimization.h>
#else
#include <GEL/HMesh/Manifold.h>
#include <GEL/HMesh/mesh_optimization.h>
#endif


namespace DSC2D {
    
    
    /**
     The base class representing a simplicial complex.
     */
    class DeformableSimplicialComplex
    {
        friend class ObjectGenerator;
    public:
        enum LABEL_OPT {NO_LABEL = -3, OUTSIDE = 0, INTERFACE = -1, CROSSING = -2};
        enum OBJECTS_TYPE {FILLED_HALF, FILLED, SQUARE, BLOB, BLOBS};
        
        typedef HMesh::VertexID node_key;
        typedef HMesh::HalfEdgeID edge_key;
        typedef HMesh::FaceID face_key;
        
        /**
         Used for max_min_angle optimization.
         */
        struct PQElem
        {
            real pri;
            edge_key h;
            int time;
            
            PQElem(real _pri, edge_key _h, int _time):
            pri(_pri), h(_h), time(_time) {}
        };
        
    protected:
        
        real AVG_LENGTH;
        real AVG_AREA;
        
        real DEG_ANGLE;
        real MIN_ANGLE;
        real COS_MIN_ANGLE;
        
        real DEG_AREA;
        real MIN_AREA;
        real MAX_AREA;
        
        real DEG_LENGTH;
        real MIN_LENGTH;
        real MAX_LENGTH;
        
        vec3 INTERFACE_COLOR;
        vec3 OUTSIDE_COLOR;
        vec3 CROSSING_COLOR;
        vec3 DEFAULT_COLOR;
        vec3 OUTSIDE_FACE_COLOR;
        vec3 DEFAULT_FACE_COLOR;
        
    private:
        HMesh::Manifold *mesh;
        DesignDomain *design_domain;
        
        HMesh::VertexAttributeVector<vec2> destination;
        
        HMesh::VertexAttributeVector<int> vertex_labels;
        HMesh::HalfEdgeAttributeVector<int> edge_labels;
        HMesh::FaceAttributeVector<int> face_labels;
        
        //************** INITIALISATION ***************
    public:
        
        /**
         Creates a simplicial complex with size (SIZE_X_, SIZE_Y_). The input parameters specifies the design domain, the initial object(s) and the discretization. The latter is defined by the parameter AVG_EDGE_LENGTH, which tells how long edges are on average.
         */
        DeformableSimplicialComplex(real AVG_EDGE_LENGTH_, const std::vector<real>& points, const std::vector<int>& faces, DesignDomain *domain = nullptr);
        
        virtual ~DeformableSimplicialComplex()
        {
            mesh->clear();
        }
        
    private:
        /**
         Creates the simplicial complex.
         */
        void create_simplicial_complex(const std::vector<real>& points, const std::vector<int>& faces);
        
        //************** DISPLAY FUNCTIONS ***************
    public:
        /**
         Returns the vertex colors.
         */
        virtual HMesh::VertexAttributeVector<vec3> get_vertex_colors() const;
        
        /**
         Returns the edge colors.
         */
        virtual HMesh::HalfEdgeAttributeVector<vec3> get_edge_colors() const;
        
        /**
         Returns the face colors.
         */
        virtual HMesh::FaceAttributeVector<vec3> get_face_colors() const;
        
        //************** ATTRIBUTE FUNCTIONS ***************
    protected:
        
        /**
         Clean up the attribute vectors (the lists that stores the attributes of each vertex, edge and face). Should be called after removing primitives.
         */
        virtual void cleanup_attributes(HMesh::IDRemap& cleanup_map);
        
        /**
         Initializes the attribute vectors.
         */
        virtual void init_attributes();
        
        /**
         Initialise the attribute vectors for the vertex with ID vid. Should be called after a new vertex has been created.
         */
        virtual void init_attributes(node_key vid);
        
        /**
         Initialise the attribute vectors for the edge with ID eid. Should be called after a new edge has been created.
         */
        virtual void init_attributes(edge_key eid)
        {
            
        }
        
        /**
         Initialise the attribute vectors for the face with ID fid. Should be called after a new face has been created.
         */
        virtual void init_attributes(face_key fid, int label = NO_LABEL);
        
        /**
         Updates the attributes of the vertex with ID vid.
         */
        virtual void update_attributes(node_key vid, int label = NO_LABEL);
        
        /**
         Updates the attributes of the edge with ID heid.
         */
        virtual void update_attributes(edge_key heid, int label1 = NO_LABEL, int label2 = NO_LABEL);
        
        /**
         Updates the attributes of the face with ID fid.
         */
        virtual void update_attributes(face_key fid, int label = NO_LABEL);
        
        /**
         Updates the attributes of the faces having the vertex with ID vid as a vertex.
         */
        virtual void update_locally(node_key vid);
        
    public:
        /**
         Updates face, edge and vertex attributes.
         */
        void update_attributes();
        
        
        //************** GETTERS ***************
    public:
        
        /**
         Returns the average edge length of the edges in the simplical complex.
         */
        real get_avg_edge_length() const
        {
            return AVG_LENGTH;
        }
        
        /**
         Returns the approximate center of the simplicial complex.
         */
        vec2 get_center();
        
        /**
         Returns the total volume of the simplicial complex.
         */
        real get_volume() const
        {
            return design_domain->get_volume();
        }
        
        /**
         Returns the design domain.
         */
        const DesignDomain* get_design_domain() const
        {
            return design_domain;
        }
        
        /**
         Returns the number of vertices.
         */
        int get_no_vertices() const
        {
            return static_cast<int>(mesh->no_vertices());
        }
        
        /**
         Returns the number of edges.
         */
        int get_no_halfedges() const
        {
            return static_cast<int>(mesh->no_halfedges());
        }
        
        /**
         Returns the number of faces.
         */
        int get_no_faces() const
        {
            return static_cast<int>(mesh->no_faces());
        }
        
        /**
         Returns the position of the vertex with ID vid.
         */
        vec2 get_pos(node_key vid) const;
        
        /**
         Returns the positions of the vertices of the face with ID fid.
         */
        std::vector<vec2> get_pos(face_key fid) const;
        
        /**
         Returns the new position of the vertex with ID vid.
         */
        vec2 get_destination(node_key vid) const;
        
        /**
         Returns the new positions of the vertices of the face with ID fid.
         */
        std::vector<vec2> get_destination(face_key fid) const;
        
        /**
         Returns the IDs of the neighbouring vertices of the vertex with ID vid. If the interface parameter is true, it only returns the neighbouring vertices which are also interface.
         */
        std::vector<node_key> get_verts(node_key vid, bool interface = false) const;
        
        /**
         Returns the IDs of the vertices of the face with ID fid.
         */
        std::vector<node_key> get_verts(face_key fid) const;
        
        /**
         Returns the IDs of the edges of the face with ID fid.
         */
        std::vector<edge_key> get_edges(face_key fid) const;
        
        /**
         Returns the label of the face with ID fid.
         */
        int get_label(face_key fid) const
        {
            return face_labels[fid];
        }
        
        /**
         Returns the label of the edge with ID eid.
         */
        int get_label(edge_key eid) const
        {
            return edge_labels[eid];
        }
        
        /**
         Returns the label of the vertex with ID vid.
         */
        int get_label(node_key vid) const
        {
            return vertex_labels[vid];
        }
        
        
        /**
         Returns sorted labels of the neighbouring faces of the interface vertex with ID vid.
         */
        std::vector<int> get_interface_labels(node_key vid) const;
        
        
        //************** SETTERS ***************
    protected:
        
        /**
         Sets the position of the vertex with ID vid to p. Should only be used internally by the simplicial complex class.
         */
        void set_pos(node_key vid, vec2 p);
        
    public:
        /**
         Sets the destination of the vertex with ID vid to dest.
         To actually move the vertices to their destination, call the deform function.
         */
        virtual void set_destination(const node_key& vid, const vec2& dest);
        
        /**
         Sets the average edge length of the edges in the simplical complex.
         */
        void set_avg_edge_length(real avg_edge_length)
        {
            AVG_LENGTH = avg_edge_length;
            AVG_AREA = 0.5*std::sqrt(3./4.)*avg_edge_length*avg_edge_length;
        }
        
        //************** ITERATORS ***************
    public:
        HMesh::VertexIDIterator vertices_begin() const
        {
            return mesh->vertices_begin();
        }
        
        HMesh::VertexIDIterator vertices_end() const
        {
            return mesh->vertices_end();
        }
        
        HMesh::HalfEdgeIDIterator halfedges_begin() const
        {
            return mesh->halfedges_begin();
        }
        
        HMesh::HalfEdgeIDIterator halfedges_end() const
        {
            return mesh->halfedges_end();
        }
        
        HMesh::FaceIDIterator faces_begin() const
        {
            return mesh->faces_begin();
        }
        
        HMesh::FaceIDIterator faces_end() const
        {
            return mesh->faces_end();
        }
        
        HMesh::Walker walker(node_key vid) const
        {
            return mesh->walker(vid);
        }
        
        HMesh::Walker walker(edge_key eid) const
        {
            return mesh->walker(eid);
        }
        
        HMesh::Walker walker(face_key fid) const
        {
            return mesh->walker(fid);
        }
        
        //************** PRECONDITIONS ***************
    public:
        /**
         Returns whether the vertex with ID vid is situated at a crossing of interfaces.
         */
        bool is_crossing(node_key vid)
        {
            return vertex_labels[vid] == CROSSING;
        }
        
        /**
         Returns whether the vertex with ID vid is situated at a crossing of interfaces.
         */
        bool is_crossing(node_key vid) const
        {
            return vertex_labels[vid] == CROSSING;
        }
        
        /**
         Returns whether the vertex with ID vid is a part of the interface.
         */
        bool is_interface(node_key vid)
        {
            return vertex_labels[vid] == INTERFACE;
        }
        
        /**
         Returns whether the vertex with ID vid is a part of the interface.
         */
        bool is_interface(node_key vid) const
        {
            return vertex_labels[vid] == INTERFACE;
        }
        
        /**
         Returns whether the edge with ID eid is a part of the interface.
         */
        bool is_interface(edge_key eid)
        {
            return edge_labels[eid] == INTERFACE;
        }
        
        /**
         Returns whether the edge with ID eid is a part of the interface.
         */
        bool is_interface(edge_key eid) const
        {
            return edge_labels[eid] == INTERFACE;
        }
        
        /**
         Returns whether the vertex with ID vid is outside.
         */
        bool is_outside(node_key vid)
        {
            return vertex_labels[vid] == OUTSIDE;
        }
        
        /**
         Returns whether the vertex with ID vid is outside.
         */
        bool is_outside(node_key vid) const
        {
            return vertex_labels[vid] == OUTSIDE;
        }
        
        /**
         Returns whether the edge with ID eid is outside.
         */
        bool is_outside(edge_key eid)
        {
            return edge_labels[eid] == OUTSIDE;
        }
        
        /**
         Returns whether the edge with ID eid is outside.
         */
        bool is_outside(edge_key eid) const
        {
            return edge_labels[eid] == OUTSIDE;
        }
        
        /**
         Returns whether the face with ID fid is outside.
         */
        bool is_outside(face_key fid)
        {
            return get_label(fid) == OUTSIDE;
        }
        
        /**
         Returns whether the face with ID fid is outside.
         */
        bool is_outside(face_key fid) const
        {
            return get_label(fid) == OUTSIDE;
        }
        
        /**
         Returns whether the vertex with ID vid is movable, i.e. interface and unsafe editable.
         */
        virtual bool is_movable(node_key vid) const;
        
        /**
         Returns whether the edge with ID eid is movable, i.e. interface and unsafe editable.
         */
        virtual bool is_movable(edge_key eid) const;
        
    protected:
        
        /**
         Returns whether the vertex with ID vid is editable, but not safe to interface changes i.e. if you edit the vertex you may change the interface.
         */
        virtual bool unsafe_editable(node_key vid) const;
        
        /**
         Returns whether the vertex with ID vid is completely safe to edit.
         */
        virtual bool safe_editable(node_key vid) const;
        
        /**
         Returns whether the edge with ID eid is editable, but not safe to interface changes i.e. if you edit the edge you may change the interface.
         */
        virtual bool unsafe_editable(edge_key eid) const;
        
        /**
         Returns whether the edge with ID eid is completely safe to edit.
         */
        virtual bool safe_editable(edge_key eid) const;
        
        //************** MESH FUNCTIONS ***************
    public:
        /**
         Move the vertices in the simplicial complex to their new position. The new position is set by the update_pos function.
         */
        void deform();
        
    private:
        
        /**
         Moves a the vertex with ID vid to its new position. Returns whether the vertex was succesfully moved to its new position.
         */
        bool move_vertex(node_key vid);
        
        /**
         Splits the face fid by inserting a vertex at the barycenter of the face. Returns whether it suceeds or not.
         */
        bool split(face_key fid);
        
        /**
         Splits the edge eid by inserting a vertex at the center of the edge and splitting the two neighbouring faces of the edge. Returns whether it suceeds or not.
         */
        bool split(edge_key eid);
        
        /**
         Collapses the edge with ID eid and updates attributes. If safe is true, the collapse will affect the shape of the interface minimally. Returns whether it suceeds or not.
         */
        bool collapse(const edge_key& eid, bool safe);
        
        /**
         Collapses the face with ID fid and updates attributes. If safe is true, the collapse will affect the shape of the interface minimally. Returns whether it suceeds or not.
         */
        bool collapse(const face_key& fid, bool safe);
        
        /**
         Collapses the edge with pointed to by the walker hew and updates attributes. The weight determines position and destination of the surviving node which are a weight between the two original nodes. Returns whether it suceeds or not.
         */
        bool collapse(HMesh::Walker hew, real weight);
        
        
        //************** QUALITY CONTROL ***************
        
    protected:
        /**
         Improves the quality of the simplicial complex by smoothing, removing needles and caps, maximize the minimum angle and removing degenerate faces.
         */
        void fix_complex();
        
    private:
        
        std::vector<edge_key> sorted_face_edges(face_key fid);
        
        void add_one_ring_to_queue(HMesh::HalfEdgeAttributeVector<int>& touched, std::priority_queue<PQElem>& Q, node_key v, const HMesh::EnergyFun& efun);
        
        void add_to_queue(HMesh::HalfEdgeAttributeVector<int>& touched, std::priority_queue<PQElem>& Q, edge_key h, const HMesh::EnergyFun& efun);
        
        void priority_queue_optimization(const HMesh::EnergyFun& efun);
        
    private:
        /**
         Maximize minimum angles using greedy approach by flipping edges.
         */
        void max_min_angle();
        
        /**
         Performs Laplacian smoothing on all safe editable vertices.
         */
        void smooth(real t = 1.);
        
        /**
         Remove needles, triangles with one very short edge, by splitting the face (inserting a vertex at the barycenter of the face).
         */
        void remove_needles();
        
        /**
         Remove caps, triangles with one very large angle, by flipping edges.
         */
        void remove_caps();
        
        /**
         Remove degenerate edges.
         */
        void remove_degenerate_edges();
        
        /**
         Remove degenerate faces.
         */
        void remove_degenerate_faces();
        
        //************** DETAIL CONTROL ***************
        
    public:
        /**
         Resize the elements of the simplicial complex by thinning, thickening and splitting of the interface.
         */
        void resize_complex();
        
    private:
        /**
         Split interface edges that are too long.
         */
        void thickening_interface();
        
        /**
         Collapse interface edges that are too long.
         */
        void thinning_interface();
        
        /**
         Remove any vertex as long as the min angles of the resulting triangulation
         are not too bad.
         */
        void thinning();
        
        /**
         Inserts vertices where faces are too big.
         */
        void thickening();
        
        //************** UTIL ***************
    private:
        
        /**
         Checks that the mesh is valid, i.e. that each face has three vertices and that the area of a face is positive (the face is not degenerate).
         */
        void validity_check();
        
        /**
         Returns the minimum angle of the face with ID fid.
         */
        real min_angle(face_key fid);
        
        /**
         Returns the minimum edge length of the edges of the face with ID fid.
         */
        real min_edge_length(face_key fid);
        
        /**
         * Returns the new minimum face quality (minimum angle) when moving a node from old_pos to new_pos. The edges in the link of the node should be passed in eids.
         */
        real min_quality(const std::vector<edge_key>& eids, const vec2& pos_old, const vec2& pos_new);
        
        /**
         Returns whether the half edge is possible to collapse.
         */
        bool is_collapsable(HMesh::Walker hew, bool safe);
        
    public:
        /**
         Returns the length of the edge with ID eid.
         */
        real length(edge_key eid) const;
        
        /**
         Returns the length of the edge with ID eid when using the new positions.
         */
        real length_new(edge_key eid) const;
        
        /**
         Calculates the area of the face with ID fid.
         */
        real area(face_key fid) const;
        
        /**
         Calculates the area of the face with ID fid when using the new positions.
         */
        real area_destination(face_key fid) const;
        
        /**
         Returns the positions of the optimization variables (the movable interface nodes).
         */
        std::vector<vec2> get_design_variable_positions();
        
        /**
         Returns the positions of the interface edges.
         */
        std::vector<vec2> get_interface_edge_positions();
        
        /**
         Returns the maximum distance a vertex is supposed to be moved (the distance between its new and old position).
         Should be called before move_vertices().
         */
        virtual real max_move_distance() const;
        
        /**
         Calculates the average position of the neighbouring vertices to the vertex with ID vid.
         If interface is true, the average position is only calculated among the neighbouring vertices that are interface.
         */
        vec2 get_barycenter(node_key vid, bool interface) const;
        
        /**
         For a given walker on a interface edge returnes walker moved to the next edge allong the interface. 
         It follows (also in crossings) the interface of the single label (defined by the face of the given edge).
         NOTE: is not an atomic operation (no_steps is incremented more than once), but full_circle can be used, 
         i.e. to walk on the perimeter of the object.
         */
        HMesh::Walker next_interface(HMesh::Walker hid) const;
        
        /**
         For a given walker on a interface edge returnes walker moved to previous edge allong the interface. 
         It follows (also in crossings) the interface of the single label (defined by the face of the given edge).
         NOTE: is not an atomic operation (no_steps is incremented more than once), but full_circle can be used,
         i.e. to walk on the perimeter of the object.
         */
        HMesh::Walker previous_interface(HMesh::Walker hid) const; 
        
        /**
         Calculates the filtered position of an interface or crossing vertex with ID vid. Input filter contains weights
         applied to all vertices along all interfaces with distance 0,1,2.. from ID, which are subsequently averaged.
         E.g. filtering with [0 1] is the same as avg_pos, but works also on crossings.
         */
        vec2 filter_vertex(node_key vid, std::vector<real> &filter) const;
        
        /**
         Computes the normal of the interface vertex with ID vid.
         If the vertex is not on the interface, the function returns a zero vector.
         */
        vec2 get_normal(node_key vid) const;
        
        /**
         Computes the normal of the interface vertex with ID nid using destinations instead of positions.
         If the vertex is not on the interface, the function returns a zero vector.
         */
        vec2 get_normal_destination(node_key nid) const;
        
        /**
         Clamps the position of the vertex with ID vid plus the vector vec to be within the design domain by scaling the vector v.
         */
        void clamp_vector(const node_key& vid, vec2& vec) const;
        
        /**
         Calculates the intersection with the link of the vertex with ID vid and the line from the position of this vertex towards destination and to infinity. It returns t which is where on the line the intersection occurs. If t=0, the interesection occured at the position of the vertex with ID vid or never occured. If t=1 the intersection occured at the destination point. If 0<t<1 the intersection occured between these points. Finally, if t>1 the intersection occured farther away from the vertex position than destination.
         */
        real intersection_with_link(const node_key& vid, vec2 destination) const;
        
    };
    
}