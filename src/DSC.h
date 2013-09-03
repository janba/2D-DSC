//
//  simplicial_complex.h
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 9/27/12.
//  Copyright (c) 2012 DTU Informatics. All rights reserved.
//

#ifndef _DDSC_mesh_h
#define _DDSC_mesh_h

#include <queue>
#include "util.h"

#ifdef WIN32
#include <HMesh/Manifold.h>
#include <HMesh/mesh_optimization.h>
#else
#include <GEL/HMesh/Manifold.h>
#include <GEL/HMesh/mesh_optimization.h>
#endif

/**
 A domain class for specifying for example the design domain.
 */
class Domain
{
    std::vector<CGLA::Vec2d> corners; // Specified in a clockwise order
    double volume;
    int label;
    
public:
    Domain() {}
    
    /**
     Creates a domain defined by the corners given as input. The corners should be specified in a clockwise order. It is possible to specify a boundary gap which translates the entire domain by the amount specified by the second input parameter. It is also possible to specify a label of the domain (used for easy creation of objects).
     */
    Domain(std::vector<CGLA::Vec2d> _corners, double boundary = 0., int _label = 1): corners(_corners), volume(-1.), label(_label)
    {
        for(std::vector<CGLA::Vec2d>::iterator ci = corners.begin(); ci != corners.end(); ci++)
        {
            (*ci)[0] = (*ci)[0] + boundary;
            (*ci)[1] = (*ci)[1] + boundary;
        }
    }
    
    /**
     Returns the corners of the design domain.
     */
    std::vector<CGLA::Vec2d> get_corners() const;
    
    /**
     Returns an approximate center of the design domain.
     */
    CGLA::Vec2d get_center();
    
    /**
     Returns the label associated with the domain.
     */
    int get_label();
    
    /**
     Returns the total volume of the domain.
     */
    double get_volume();
    
    /**
     Clamps the position pos to be within the domain.
     */
    void clamp_position(CGLA::Vec2d& p) const;
    
    /**
     Clamps p + v to be within the domain by scaling the vector v if p is inside the domain. The position p is therefore not garanteed to be within the domain.
     */
    void clamp_vector(const CGLA::Vec2d& p, CGLA::Vec2d& v) const;
    
    /**
     Returns whether the position p is inside the domain.
     */
    bool is_inside(CGLA::Vec2d p) const;
};

/**
 Extends the Domain class to be able to relate an attribute to the domain.
 */
template <class att_type>
class AttributeDomain : public Domain
{
    att_type att;
    
public:
    AttributeDomain(std::vector<CGLA::Vec2d> _corners, att_type _att): Domain(_corners), att(_att)
    {
        
    }
    
    /**
     Returns the attribute of the domain.
     */
    att_type get_att()
    {
        return att;
    }
    
};

/**
 Used for max_min_angle optimization.
 */
struct PQElem
{
    double pri;
    HMesh::HalfEdgeID h;
    int time;
    
    PQElem(double _pri, HMesh::HalfEdgeID _h, int _time):
    pri(_pri), h(_h), time(_time) {}
};


/**
 The base class representing a simplicial complex.
 */
class DeformableSimplicialComplex
{
public:
    enum LABEL_OPT {NO_LABEL = -3, OUTSIDE = 0, INTERFACE = -1, CROSSING = -2};
    enum DESIGN_DOMAIN_TYPE {RECTANGLE, L, ESO};
    enum OBJECTS_TYPE {FILLED_HALF, FILLED, SQUARE, BLOB, BLOBS};
    
protected:
    double AVG_EDGE_LENGTH;
    double BOUNDARY_GAP;
    int SIZE_X;
    int SIZE_Y;
    
    double DEG_ANGLE;
    double MIN_ANGLE;
    double COS_MIN_ANGLE;
    
    double DEG_AREA;
    double MIN_AREA;
    double MAX_AREA;
    
    double DEG_EDGE_LENGTH;
    double MIN_EDGE_LENGTH;
    double MAX_EDGE_LENGTH;
    
    double MIN_DEFORMATION;
    
    CGLA::Vec3d INTERFACE_COLOR;
    CGLA::Vec3d OUTSIDE_COLOR;
    CGLA::Vec3d CROSSING_COLOR;
    CGLA::Vec3d DEFAULT_COLOR;
    CGLA::Vec3d OUTSIDE_FACE_COLOR;
    CGLA::Vec3d DEFAULT_FACE_COLOR;
    
private:
    HMesh::Manifold *mesh;
    Domain *design_domain;
    std::vector<Domain*> object_domains;
    
    HMesh::VertexAttributeVector<CGLA::Vec2d> new_pos;
    
    HMesh::VertexAttributeVector<int> vertex_labels;
    HMesh::HalfEdgeAttributeVector<int> edge_labels;
    HMesh::FaceAttributeVector<int> face_labels;
    
    //************** INITIALISATION ***************
public:
    
    /**
     Creates a simplicial complex with size (SIZE_X_, SIZE_Y_). The input parameters specifies the design domain, the initial object(s) and the discretization. The latter is defined by the parameter AVG_EDGE_LENGTH, which tells how long edges are on average.
     */
    DeformableSimplicialComplex(int SIZE_X_, int SIZE_Y_, DESIGN_DOMAIN_TYPE design_domain, OBJECTS_TYPE obj, double AVG_EDGE_LENGTH_ = 25.);
    
    virtual ~DeformableSimplicialComplex()
    {
        mesh->clear();
    }
    
private:
    /**
     Creates the simplicial complex.
     */
    void create_simplicial_complex();
    
    /**
     Creates the design domain.
     */
    void create_design_domain(DESIGN_DOMAIN_TYPE design_domain);
    
    /**
     Creates the object domains.
     */
    void create_object_domains(OBJECTS_TYPE obj);
    
    /**
     Creates objects.
     */
    void create_objects();
    
    /**
     Fit the interface exactly to the object domains.
     */
    void fit_to_objects();
    
    //************** DISPLAY FUNCTIONS ***************
public:
    /**
     Returns the vertex colors.
     */
    virtual HMesh::VertexAttributeVector<CGLA::Vec3d> get_vertex_colors() const;

    /**
     Returns the edge colors.
     */
    virtual HMesh::HalfEdgeAttributeVector<CGLA::Vec3d> get_edge_colors() const;
    
    /**
     Returns the face colors.
     */
    virtual HMesh::FaceAttributeVector<CGLA::Vec3d> get_face_colors() const;
    
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
    virtual void init_attributes(HMesh::VertexID vid);
    
    /**
     Initialise the attribute vectors for the edge with ID eid. Should be called after a new edge has been created.
     */
    virtual void init_attributes(HMesh::HalfEdgeID eid)
    {
        
    }
    
    /**
     Initialise the attribute vectors for the face with ID fid. Should be called after a new face has been created.
     */
    virtual void init_attributes(HMesh::FaceID fid, int label = NO_LABEL);
    
    /**
     Updates the attributes of the vertex with ID vid.
     */
    virtual void update_attributes(HMesh::VertexID vid, int label = NO_LABEL);
    
    /**
     Updates the attributes of the edge with ID heid.
     */
    virtual void update_attributes(HMesh::HalfEdgeID heid, int label1 = NO_LABEL, int label2 = NO_LABEL);
    
    /**
     Updates the attributes of the face with ID fid.
     */
    virtual void update_attributes(HMesh::FaceID fid, int label = NO_LABEL);
    
    /**
     Updates the attributes of the faces having the vertex with ID vid as a vertex.
     */
    virtual void update_locally(HMesh::VertexID vid);
    
public:
    /**
     Updates face, edge and vertex attributes.
     */
    void update_attributes();
    
    
    //************** GETTERS ***************
public:
    /**
     Returns the size of the simplicial complex in the x-direction.
     */
    int get_size_x() const
    {
        return SIZE_X;
    }
    
    /**
     Returns the size of the simplicial complex in the y-direction.
     */
    int get_size_y() const
    {
        return SIZE_Y;
    }
    
    /**
     Returns the average edge length of the edges in the simplical complex.
     */
    double get_avg_edge_length() const
    {
        return AVG_EDGE_LENGTH;
    }
	
    /**
     Returns the boundary gap.
     */
	double get_boundary_gap() const
    {
        return BOUNDARY_GAP;
    }

    /**
     Returns the minimum deformation possible.
     */
    double get_min_deformation() const
    {
        return MIN_DEFORMATION;
    }
    
    /**
     Returns the width of the simplicial complex.
     */
    double get_width() const;
    
    /**
     Returns the height of the simplicial complex.
     */
    double get_height() const;
    
    /**
     Returns the approximate center of the simplicial complex.
     */
    CGLA::Vec2d get_center();
    
    /**
     Returns the total volume of the simplicial complex.
     */
    double get_volume() const
    {
        return design_domain->get_volume();
    }
    
    /**
     Returns the design domain.
     */
    const Domain* get_design_domain() const
    {
        return design_domain;
    }
    
    /**
     Returns the object domains.
     */
    std::vector<Domain*> get_object_domains() const
    {
        return object_domains;
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
    CGLA::Vec2d get_pos(HMesh::VertexID vid) const;
    
    /**
     Returns the positions of the vertices of the face with ID fid.
     */
    std::vector<CGLA::Vec2d> get_pos(HMesh::FaceID fid) const;
        
    /**
     Returns the new position of the vertex with ID vid.
     */
    CGLA::Vec2d get_pos_new(HMesh::VertexID vid) const;
    
    /**
     Returns the new positions of the vertices of the face with ID fid.
     */
    std::vector<CGLA::Vec2d> get_pos_new(HMesh::FaceID fid) const;
    
    /**
     Returns the IDs of the neighbouring vertices of the vertex with ID vid. If the interface parameter is true, it only returns the neighbouring vertices which are also interface.
     */
    std::vector<HMesh::VertexID> get_verts(HMesh::VertexID vid, bool interface = false) const;
    
    /**
     Returns the IDs of the vertices of the face with ID fid.
     */
    std::vector<HMesh::VertexID> get_verts(HMesh::FaceID fid) const;
    
    /**
     Returns the IDs of the edges of the face with ID fid.
     */
    std::vector<HMesh::HalfEdgeID> get_edges(HMesh::FaceID fid) const;
    
	/**
     Returns the label of the face with ID fid.
     */
    int get_label(HMesh::FaceID fid) const
    {
        return face_labels[fid];
    }

	 /**
     Returns the label of the edge with ID eid.
     */
    int get_label(HMesh::HalfEdgeID eid) const
    {
        return edge_labels[eid];
    }

	 /**
     Returns the label of the vertex with ID vid.
     */
    int get_label(HMesh::VertexID vid) const
    {
        return vertex_labels[vid];
    }


	/**
     Returns sorted labels of the neighbouring faces of the interface vertex with ID vid. 
     */
    std::vector<int> get_interface_labels(HMesh::VertexID vid) const;
    
    
    //************** SETTERS ***************
private:
    
    /**
     Sets the position of the vertex with ID vid to p. Should only be used internally by the simplicial complex class.
     */
    void set_pos(HMesh::VertexID vid, CGLA::Vec2d p);
    
public:
    /**
     Sets the destination of the vertex with ID vid to dest.
     To actually move the vertices to their destination, call the deform function.
     */
    virtual void set_destination(HMesh::VertexID vid, CGLA::Vec2d dest);
    
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
    
    HMesh::Walker walker(HMesh::VertexID vid) const
    {
        return mesh->walker(vid);
    }
    
    HMesh::Walker walker(HMesh::HalfEdgeID eid) const
    {
        return mesh->walker(eid);
    }
    
    HMesh::Walker walker(HMesh::FaceID fid) const
    {
        return mesh->walker(fid);
    }
    
    //************** PRECONDITIONS ***************
public:
    /**
     Returns whether the vertex with ID vid is situated at a crossing of interfaces.
     */
    bool is_crossing(HMesh::VertexID vid)
    {
        return vertex_labels[vid] == CROSSING;
    }
    
    /**
     Returns whether the vertex with ID vid is situated at a crossing of interfaces.
     */
    bool is_crossing(HMesh::VertexID vid) const
    {
        return vertex_labels[vid] == CROSSING;
    }
    
    /**
     Returns whether the vertex with ID vid is a part of the interface.
     */
    bool is_interface(HMesh::VertexID vid)
    {
        return vertex_labels[vid] == INTERFACE;
    }
    
    /**
     Returns whether the vertex with ID vid is a part of the interface.
     */
    bool is_interface(HMesh::VertexID vid) const
    {
        return vertex_labels[vid] == INTERFACE;
    }
    
    /**
     Returns whether the edge with ID eid is a part of the interface.
     */
    bool is_interface(HMesh::HalfEdgeID eid)
    {
        return edge_labels[eid] == INTERFACE;
    }
    
    /**
     Returns whether the edge with ID eid is a part of the interface.
     */
    bool is_interface(HMesh::HalfEdgeID eid) const
    {
        return edge_labels[eid] == INTERFACE;
    }
    
    /**
     Returns whether the vertex with ID vid is outside.
     */
    bool is_outside(HMesh::VertexID vid)
    {
        return vertex_labels[vid] == OUTSIDE;
    }
    
    /**
     Returns whether the vertex with ID vid is outside.
     */
    bool is_outside(HMesh::VertexID vid) const
    {
        return vertex_labels[vid] == OUTSIDE;
    }
    
    /**
     Returns whether the edge with ID eid is outside.
     */
    bool is_outside(HMesh::HalfEdgeID eid) 
    {
        return edge_labels[eid] == OUTSIDE;
    }
    
    /**
     Returns whether the edge with ID eid is outside.
     */
    bool is_outside(HMesh::HalfEdgeID eid) const
    {
        return edge_labels[eid] == OUTSIDE;
    }
    
    /**
     Returns whether the face with ID fid is outside.
     */
    bool is_outside(HMesh::FaceID fid)
    {
        return get_label(fid) == OUTSIDE;
    }
    
    /**
     Returns whether the face with ID fid is outside.
     */
    bool is_outside(HMesh::FaceID fid) const 
    {
        return get_label(fid) == OUTSIDE;
    }
    
    /**
     Returns whether the vertex with ID vid is movable, i.e. interface and unsafe editable.
     */
    virtual bool is_movable(HMesh::VertexID vid) const;
    
    /**
     Returns whether the edge with ID eid is movable, i.e. interface and unsafe editable.
     */
    virtual bool is_movable(HMesh::HalfEdgeID eid) const;
    
protected:
    
    /**
     Returns whether the vertex with ID vid is editable, but not safe to interface changes i.e. if you edit the vertex you may change the interface.
     */
    virtual bool unsafe_editable(HMesh::VertexID vid) const;
    
    /**
     Returns whether the vertex with ID vid is completely safe to edit.
     */
    virtual bool safe_editable(HMesh::VertexID vid) const;
    
    /**
     Returns whether the edge with ID eid is editable, but not safe to interface changes i.e. if you edit the edge you may change the interface.
     */
    virtual bool unsafe_editable(HMesh::HalfEdgeID eid) const;
    
    /**
     Returns whether the edge with ID eid is completely safe to edit.
     */
    virtual bool safe_editable(HMesh::HalfEdgeID eid) const;
    
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
    bool move_vertex(HMesh::VertexID vid);
    
    /**
     Splits the face fid by inserting a vertex at the barycenter of the face. Returns whether it suceeds or not.
     */
    bool split(HMesh::FaceID fid);
    
    /**
     Splits the edge eid by inserting a vertex at the center of the edge and splitting the two neighbouring faces of the edge. Returns whether it suceeds or not.
     */
    bool split(HMesh::HalfEdgeID eid);
    
    /**
     Safely collapses the halfedge with ID heid and updates attributes. Returns whether it suceeds or not.
     */
    bool safe_collapse(HMesh::HalfEdgeID eid);
    
    /**
     Collapses the halfedge with ID heid and updates attributes. Returns whether it suceeds or not. Note that this method may edit the interface.
     */
    bool unsafe_collapse(HMesh::HalfEdgeID heid);
    
    
    //************** QUALITY CONTROL ***************
    
protected:
    /**
     Improves the quality of the simplicial complex by smoothing, removing needles and caps, maximize the minimum angle and removing degenerate faces.
     */
    void fix_mesh();
    
private:
    
    std::vector<HMesh::HalfEdgeID> sorted_face_edges(HMesh::FaceID fid);
    
    void add_one_ring_to_queue(HMesh::HalfEdgeAttributeVector<int>& touched, std::priority_queue<PQElem>& Q, HMesh::VertexID v, const HMesh::EnergyFun& efun);
    
    void add_to_queue(HMesh::HalfEdgeAttributeVector<int>& touched, std::priority_queue<PQElem>& Q, HMesh::HalfEdgeID h, const HMesh::EnergyFun& efun);
    
    void priority_queue_optimization(const HMesh::EnergyFun& efun);
    
private:
    /**
     Maximize minimum angles using greedy approach by flipping edges.
     */
    void max_min_angle();
    
    /**
     Performs Laplacian smoothing on all safe editable vertices.
     */
    void smooth(double t = 1.);
    
    /**
     Remove needles, triangles with one very short edge, by splitting the face (inserting a vertex at the barycenter of the face).
     */
    void remove_needles();
    
    /**
     Remove caps, triangles with one very large angle, by flipping edges.
     */
    void remove_caps();
    
    /**
     Remove degenerate faces.
     */
    void remove_degenerate_faces();
    
    //************** DETAIL CONTROL ***************
    
public:
    /**
     Resize the elements of the simplicial complex by thinning, thickening and splitting of the interface.
     */
    void resize_elements();
    
private:
    /**
     Split interface edges that are too long.
     */
    bool split_interface();
    
    /**
     Collapse interface edges that are too long.
     */
    bool collapse_interface();
    
    /**
     Remove any vertex as long as the min angles of the resulting triangulation
     are not too bad.
     */
    bool thinning();
    
    /**
     Inserts vertices where faces are too big.
     */
    bool thickening();
    
    //************** UTIL ***************
private:
    
    /**
     Returns the minimum angle of the face with ID fid.
     */
    double min_angle(HMesh::FaceID fid);
    
    /**
     Returns the minimum edge length of the edges of the face with ID fid.
     */
    double min_edge_length(HMesh::FaceID fid);
    
public:
    /**
     Returns the length of the edge with ID eid.
     */
    double length(HMesh::HalfEdgeID eid) const;
    
    /**
     Returns the length of the edge with ID eid when using the new positions.
     */
    double length_new(HMesh::HalfEdgeID eid) const;
    
    /**
     Calculates the area of the face with ID fid.
     */
    double area(HMesh::FaceID fid) const;
    
    /**
     Calculates the area of the face with ID fid when using the new positions.
     */
    double area_new(HMesh::FaceID fid) const;
    
    /**
     Returns the positions of the optimization variables (the movable interface nodes).
     */
    std::vector<CGLA::Vec2d> get_design_variable_positions();
    
    /**
     Returns the positions of the interface edges.
     */
    std::vector<CGLA::Vec2d> get_interface_edge_positions();
    
    /**
     Returns the maximum distance a vertex is supposed to be moved (the distance between its new and old position).
     Should be called before move_vertices().
     */
    virtual double max_move_distance() const;
    
    /**
     Calculates the average position of the neighbouring vertices to the vertex with ID vid.
     If interface is true, the average position is only calculated among the neighbouring vertices that are interface.
     */
    CGLA::Vec2d avg_pos(HMesh::VertexID vid, bool interface) const;

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
	CGLA::Vec2d filter_vertex(HMesh::VertexID vid, std::vector<double> &filter) const;
	
	/**
     Computes the normal of the interface vertex with ID vid.
     If the vertex is not on the interface, the function returns a zero vector.
     */
    CGLA::Vec2d normal(HMesh::VertexID vid) const;
    
    /**
     Clamps the position of the vertex with ID vid plus the vector vec to be within the design domain by scaling the vector v.
     */
    void clamp_vector(const HMesh::VertexID& vid, CGLA::Vec2d& vec) const;
    
    /**
     Returns whether or not the face with ID fid is inside the domain.
     */
    bool inside_domain(HMesh::FaceID fid, Domain domain) const;
    
    /**
     Calculates the intersection with the link of the vertex with ID vid and the line from the position of this vertex towards destination and to infinity. It returns t which is where on the line the intersection occurs. If t=0, the interesection occured at the position of the vertex with ID vid or never occured. If t=1 the intersection occured at the destination point. If 0<t<1 the intersection occured between these points. Finally, if t>1 the intersection occured farther away from the vertex position than destination.
     */
    double intersection_with_link(const HMesh::VertexID& vid, CGLA::Vec2d destination) const;
    
};

#endif
