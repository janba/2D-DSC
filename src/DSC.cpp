//
//  simplicial_complex.cpp
//  2D_DSC
//
//  Created by Asger Nyman Christiansen on 9/27/12.
//  Copyright (c) 2012 DTU Informatics. All rights reserved.
//

#include "DSC.h"
#include "draw.h"

void Domain::clamp_position(CGLA::Vec2d& p) const
{
    if(!is_inside(p))
    {
        CGLA::Vec2d c0, c1, b, p_proj, p_int1, p_int2;
        double length, dist1 = INFINITY, dist2 = INFINITY;
        for (int i = 0; i < corners.size(); i++)
        {
            c0 = corners[i];
            c1 = corners[(i+1)%corners.size()];
            b = CGLA::normalize(c1 - c0);
            p_proj = c0 + CGLA::dot(p - c0, b)*b;
            length = CGLA::sqr_length(p - p_proj);
            if (length < dist1)
            {
                p_int2 = p_int1;
                p_int1 = p_proj;
                dist2 = dist1;
                dist1 = length;
            }
            else if(length < dist2)
            {
                p_int2 = p_proj;
                dist2 = length;
            }
        }
        if(!is_inside(p_int1))
        {
            p = p_int1 + (p_int2 - p);
        }
        else {
            p = p_int1;
        }
    }
}

void Domain::clamp_vector(const CGLA::Vec2d& p, CGLA::Vec2d& v) const
{
    if(!is_inside(p+v))
    {
        CGLA::Vec2d c0, c1;
        double t;
        for (int i = 0; i < corners.size(); i++)
        {
            c0 = corners[i];
            c1 = corners[(i+1)%corners.size()];
            t = Util::intersection(p, v, c0, c1 - c0);
            if(t >= 0. && t < 1.)
            {
                v = t*v;
            }
        }
    }
}

bool Domain::is_inside(CGLA::Vec2d p) const
{
    return Util::is_inside(p, corners);
}

std::vector<CGLA::Vec2d> Domain::get_corners() const
{
    return corners;
}

CGLA::Vec2d Domain::get_center()
{
    CGLA::Vec2d center(0.);
    for (int i = 0; i < corners.size(); i++)
    {
        center += corners[i];
    }
    return center/static_cast<double>(corners.size());
}

int Domain::get_label()
{
    return label;
}

double Domain::get_volume()
{
    if(volume < 0.)
    {
        volume = 0.;
        std::vector<CGLA::Vec2d> c(corners);
        CGLA::Vec2d c0, c1, c2;
        while(c.size() > 2)
        {
            int i = 0;
            do {
#ifdef DEBUG
                assert(i < c.size());
#endif
                c0 = c[i];
                c1 = c[(i+1)%c.size()];
                c2 = c[(i+2)%c.size()];
                i++;
            } while (Util::is_left_of(c0,c1,c2));
            
            volume += std::abs(Util::signed_area(c0, c1, c2));
            c.erase(c.begin() + (i%c.size()));
        }
    }
    return volume;
}

DeformableSimplicialComplex::DeformableSimplicialComplex(int SIZE_X_, int SIZE_Y_, DESIGN_DOMAIN_TYPE design_domain, OBJECTS_TYPE obj, double AVG_EDGE_LENGTH_):
    SIZE_X(SIZE_X_), SIZE_Y(SIZE_Y_), AVG_EDGE_LENGTH(AVG_EDGE_LENGTH_), BOUNDARY_GAP(AVG_EDGE_LENGTH_)
{
    MIN_ANGLE = M_PI * 2./180.;
    COS_MIN_ANGLE = cos(MIN_ANGLE);
    DEG_ANGLE = 0.5*MIN_ANGLE;
    
    MAX_EDGE_LENGTH = 1.5*AVG_EDGE_LENGTH;
    MIN_EDGE_LENGTH = 0.5*AVG_EDGE_LENGTH;
    DEG_EDGE_LENGTH = 0.5*MIN_EDGE_LENGTH;
    
    MIN_DEFORMATION = DEG_EDGE_LENGTH;
    
    double avg_area = 0.5*std::sqrt(3./4.)*AVG_EDGE_LENGTH*AVG_EDGE_LENGTH;
    MAX_AREA = 1.5*avg_area;
    MIN_AREA = 0.5*avg_area;
    DEG_AREA = 0.5*MIN_AREA;
    
    INTERFACE_COLOR = DARK_RED;
    CROSSING_COLOR = RED;
    OUTSIDE_COLOR = BLACK;
    DEFAULT_COLOR = DARK_BLUE;
    OUTSIDE_FACE_COLOR = INVISIBLE;
    DEFAULT_FACE_COLOR = BLUE;
    
    create_design_domain(design_domain);
    create_object_domains(obj);
    create_simplicial_complex();
    
    new_pos = HMesh::VertexAttributeVector<CGLA::Vec2d>(get_no_vertices(), CGLA::Vec2d(0.));
    vertex_labels = HMesh::VertexAttributeVector<int>(get_no_vertices(), OUTSIDE);
    edge_labels = HMesh::HalfEdgeAttributeVector<int>(get_no_halfedges(), OUTSIDE);
    face_labels = HMesh::FaceAttributeVector<int>(get_no_faces(), OUTSIDE);
    
    create_objects();
}

void DeformableSimplicialComplex::cleanup_attributes(HMesh::IDRemap& cleanup_map)
{
    mesh->cleanup(cleanup_map);
    vertex_labels.cleanup(cleanup_map.vmap);
    edge_labels.cleanup(cleanup_map.hmap);
    face_labels.cleanup(cleanup_map.fmap);
    new_pos.cleanup(cleanup_map.vmap);
}


HMesh::VertexAttributeVector<CGLA::Vec3d> DeformableSimplicialComplex::get_vertex_colors() const
{
    HMesh::VertexAttributeVector<CGLA::Vec3d> colors = HMesh::VertexAttributeVector<CGLA::Vec3d>();
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

HMesh::HalfEdgeAttributeVector<CGLA::Vec3d> DeformableSimplicialComplex::get_edge_colors() const
{
    HMesh::HalfEdgeAttributeVector<CGLA::Vec3d> colors = HMesh::HalfEdgeAttributeVector<CGLA::Vec3d>();
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

HMesh::FaceAttributeVector<CGLA::Vec3d> DeformableSimplicialComplex::get_face_colors() const
{
    HMesh::FaceAttributeVector<CGLA::Vec3d> colors = HMesh::FaceAttributeVector<CGLA::Vec3d>();
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

bool DeformableSimplicialComplex::inside_domain(HMesh::FaceID fid, Domain domain) const
{
    for (HMesh::Walker hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
    {
        if(!domain.is_inside(get_pos(hew.vertex())))
        {
            return false;
        }
    }
    return true;
}

void create_blob(const CGLA::Vec2d& center, const double& radius, std::vector<CGLA::Vec2d>& corners)
{
    for (double a = 0; a < 2*M_PI; a += (1./32.)*2*M_PI)
    {
        corners.push_back(radius*CGLA::Vec2d(std::cos(a), -std::sin(a)) + center);
    }
}

void DeformableSimplicialComplex::create_object_domains(OBJECTS_TYPE obj)
{
    std::vector<CGLA::Vec2d> corners;
    double size;
    switch (obj) {
        case FILLED:
            object_domains.push_back(design_domain);
            break;
        case FILLED_HALF:
            size = 0.25;
            corners.push_back(CGLA::Vec2d(0., size*SIZE_Y));
            corners.push_back(CGLA::Vec2d(0., (1.-size)*SIZE_Y));
            corners.push_back(CGLA::Vec2d(SIZE_X, (1.-size)*SIZE_Y));
            corners.push_back(CGLA::Vec2d(SIZE_X, size*SIZE_Y));
            
            object_domains.push_back(new Domain(corners, BOUNDARY_GAP));
            break;
        case SQUARE:
            size = 5.;
            corners.push_back(CGLA::Vec2d(SIZE_X/size, SIZE_X/size));
            corners.push_back(CGLA::Vec2d(SIZE_X/size, (size - 1.)*SIZE_X/size));
            corners.push_back(CGLA::Vec2d((size - 1.)*SIZE_X/size, (size - 1.)*SIZE_X/size));
            corners.push_back(CGLA::Vec2d((size - 1.)*SIZE_X/size, SIZE_X/size));
            
            object_domains.push_back(new Domain(corners, BOUNDARY_GAP));
            break;
        case BLOB:
            create_blob(get_center(), std::min(SIZE_X,SIZE_Y)*0.4, corners);
            object_domains.push_back(new Domain(corners, 0.));
            break;
        case BLOBS:
        {
            // First blob
            double radius = std::min(SIZE_X,SIZE_Y)*0.1;
            CGLA::Vec2d center = get_center() + CGLA::Vec2d(0., -0.35*SIZE_Y);
            create_blob(center, radius, corners);
            object_domains.push_back(new Domain(corners, 0., 1));
            corners.clear();
            
            // Second blob
            radius = std::min(SIZE_X,SIZE_Y)*0.15;
            center = get_center() + CGLA::Vec2d(0.25*SIZE_X, -0.22*SIZE_Y);
            create_blob(center, radius, corners);
            object_domains.push_back(new Domain(corners, 0., 2));
            corners.clear();
            
            radius = std::min(SIZE_X,SIZE_Y)*0.25;
            center = get_center() + CGLA::Vec2d(0.25*SIZE_X, 0.25*SIZE_Y);
            create_blob(center, radius, corners);
            object_domains.push_back(new Domain(corners, 0., 3));
            corners.clear();
            
            radius = std::min(SIZE_X,SIZE_Y)*0.15;
            center = get_center() + CGLA::Vec2d(-0.25*SIZE_X, 0.);
            create_blob(center, radius, corners);
            object_domains.push_back(new Domain(corners, 0., 4));
        }
            break;
        default:
            break;
    }
    
}

void DeformableSimplicialComplex::create_objects()
{
    //for (int i = 0; i < 3; i++) //why?
    {
        for (HMesh::FaceIDIterator fi = faces_begin(); fi != faces_end(); fi++)
        {
            face_labels[*fi] = OUTSIDE;
            for (auto obj = object_domains.begin(); obj != object_domains.end(); obj++)
            {
                if(inside_domain(*fi, **obj))
                {
                    face_labels[*fi] = (*obj)->get_label();
                }
            }
        }
        
        init_attributes();
        update_attributes();
        
        fit_to_objects();
    }
}

void DeformableSimplicialComplex::fit_to_objects()
{
    for (auto vi = vertices_begin(); vi != vertices_end(); vi++)
    {
        if (is_movable(*vi))
        {
            CGLA::Vec2d p = get_pos(*vi);
            CGLA::Vec2d min_dir;
            double min_length = INFINITY;
            for (auto obj = object_domains.begin(); obj != object_domains.end(); obj++)
            {
                std::vector<CGLA::Vec2d> corners = (*obj)->get_corners();
                for (int i = 0; i < corners.size(); i ++)
                {
                    CGLA::Vec2d c = corners[i];
                    double l = (corners[(i+1)%corners.size()] - c).length();
                    CGLA::Vec2d b = normalize(corners[(i+1)%corners.size()] - c);
                    CGLA::Vec2d a = p - c;
                    double fac = dot(a,b);
                    CGLA::Vec2d dir = fac*b - a;
                    if (dir.length() < min_length && 0 <= fac && fac <= l)
                    {
                        min_dir = dir;
                        min_length = dir.length();
                    }
                }
            }
            if(min_length < INFINITY)
            {
                set_destination(*vi, get_pos(*vi) + min_dir);
            }
        }
    }
    
    for (auto obj = object_domains.begin(); obj != object_domains.end(); obj++)
    {
        std::vector<CGLA::Vec2d> corners = (*obj)->get_corners();
        for (int i = 0; i < corners.size(); i ++)
        {
            CGLA::Vec2d c = corners[i];
            double min_length = INFINITY;
            CGLA::Vec2d min_dir;
            HMesh::VertexID vid;
            for (auto vi = vertices_begin(); vi != vertices_end(); vi++)
            {
                if (is_movable(*vi))
                {
                    CGLA::Vec2d dir = c - get_pos(*vi);
                    if (dir.length() < min_length)
                    {
                        vid = *vi;
                        min_length = dir.length();
                        min_dir = dir;
                    }
                }
            }
            if(min_length < INFINITY)
            {
                set_destination(vid, get_pos(vid) + min_dir);
            }
        }
    }
    deform();
}

void DeformableSimplicialComplex::create_design_domain(DESIGN_DOMAIN_TYPE design)
{
    std::vector<CGLA::Vec2d> corners;
    switch (design) {
        case RECTANGLE:
            corners.push_back(CGLA::Vec2d(0.,0.));
            corners.push_back(CGLA::Vec2d(0., SIZE_Y));
            corners.push_back(CGLA::Vec2d(SIZE_X, SIZE_Y));
            corners.push_back(CGLA::Vec2d(SIZE_X, 0.));
            design_domain = new Domain(corners, BOUNDARY_GAP);
            break;
        case L:
            corners.push_back(CGLA::Vec2d(0.,0.));
            corners.push_back(CGLA::Vec2d(0., SIZE_Y));
            corners.push_back(CGLA::Vec2d(SIZE_X/2., SIZE_Y));
            corners.push_back(CGLA::Vec2d(SIZE_X/2., SIZE_Y/2.));
            corners.push_back(CGLA::Vec2d(SIZE_X, SIZE_Y/2.));
            corners.push_back(CGLA::Vec2d(SIZE_X, 0.));
            design_domain = new Domain(corners, BOUNDARY_GAP);
            break;
        case ESO:
            corners.push_back(CGLA::Vec2d(0.,0.));
            corners.push_back(CGLA::Vec2d(0., 3.*SIZE_Y/7.));
            corners.push_back(CGLA::Vec2d(30.*SIZE_X/32., 3.*SIZE_Y/7.));
            corners.push_back(CGLA::Vec2d(30.*SIZE_X/32., SIZE_Y));
            corners.push_back(CGLA::Vec2d(31.*SIZE_X/32., SIZE_Y));
            corners.push_back(CGLA::Vec2d(31.*SIZE_X/32., 3.*SIZE_Y/7.));
            corners.push_back(CGLA::Vec2d(SIZE_X, 3.*SIZE_Y/7.));
            corners.push_back(CGLA::Vec2d(SIZE_X, 0.));
            design_domain = new Domain(corners, BOUNDARY_GAP);
            break;
    }
}

/*void DeformableSimplicialComplex::create_simplicial_complex()
{
    mesh = new HMesh::Manifold();
    const double MAX_X = SIZE_X + 2*BOUNDARY_GAP;
    const double MAX_Y = SIZE_Y + 2*BOUNDARY_GAP;
    const int Nx = std::ceil(MAX_X/AVG_EDGE_LENGTH) + 1;
    const int Ny = std::ceil(MAX_Y/AVG_EDGE_LENGTH) + 1;

    std::vector<double> pts(3*Nx*Ny);
	std::vector<int> faces(2*(Nx-1)*(Ny-1));
	std::vector<int> indices;
    
    int ind;
	for(int j = 0; j < Ny; ++j)
    {
		for(int i = 0; i < Nx; ++i)
        {
            ind = 3*(i+j*Nx);
			pts[ind] = std::min(i * AVG_EDGE_LENGTH, MAX_X);
			pts[ind+1] = std::min(j * AVG_EDGE_LENGTH, MAX_Y);
			pts[ind+2] = 0.;
        }
    }
    
	for(int j = 0; j < Ny - 1; ++j)
    {
		for(int i = 0; i < Nx - 1; ++i)
        {
            int idx00 = i   + j * Nx;
            int idx10 = i+1 + j * Nx;
            int idx01 = i   + (j+1) * Nx;
            int idx11 = i+1 + (j+1) * Nx;
            
            indices.push_back(idx00);
            indices.push_back(idx10);
            indices.push_back(idx11);
            
            indices.push_back(idx00);
            indices.push_back(idx11);
            indices.push_back(idx01);
        }
    }
    
    for (int i = 0; i < faces.size(); i++)
    {
        faces[i] = 3;
    }
    
    mesh->build(pts.size()/3, &pts[0], faces.size(), &faces[0], &indices[0]);
}*/

void DeformableSimplicialComplex::create_simplicial_complex()
{
    mesh = new HMesh::Manifold();
  
	const int Nx = std::max(std::ceil(SIZE_X/AVG_EDGE_LENGTH),1.); 
    const int Ny = std::max(2*std::floor(SIZE_Y/(sqrt(3.)*AVG_EDGE_LENGTH)),2.);
	
	const double b = (double)SIZE_X/(double)Nx; // adjusted length of the triangle base
	const double h = (double)SIZE_Y/(double)Ny; // adjusted triangle height

    std::vector<double> pts;
	std::vector<int> indices;

	// POINTS
	// x positions of points on triangle basis
	std::vector<double> x_full;
	std::vector<double> x_half;

	x_full.push_back(0.);
	for (double x=AVG_EDGE_LENGTH; x<AVG_EDGE_LENGTH+(Nx+1)*b; x+=b) // stopping condition: x<= AVG_EDGE_LENGTH+Nx*b
	{
		x_full.push_back(x);
	}
	x_full.push_back(SIZE_X+2*AVG_EDGE_LENGTH);

	// x positions of points on triangle tops/bottoms
	x_half.push_back(0.); x_half.push_back(AVG_EDGE_LENGTH);
	for (double x=AVG_EDGE_LENGTH+b/2; x<AVG_EDGE_LENGTH+(Nx+1)*b-b/2; x+=b) // stopping condition: x<=AVG_EDGE_LENGTH+Nx*b-b/2
	{
		x_half.push_back(x);
	}
	x_half.push_back(SIZE_X+AVG_EDGE_LENGTH); x_half.push_back(SIZE_X+2*AVG_EDGE_LENGTH);

	for (int k = 0; k<x_full.size(); k++)
	{
		pts.push_back(x_full[k]); pts.push_back(0); pts.push_back(0); // first line of points
	}
	
	for (double y = AVG_EDGE_LENGTH; y<AVG_EDGE_LENGTH + Ny*h; y+=2*h) // stopping condition: y<=AVG_EDGE_LENGTH + (Ny-2)*h;
	{
		for (int k = 0; k<x_full.size(); k++)
		{
			pts.push_back(x_full[k]); pts.push_back(y); pts.push_back(0); // lines with points on triangle basis
		}
		for (int k = 0; k<x_half.size(); k++)
		{
			pts.push_back(x_half[k]); pts.push_back(y+h); pts.push_back(0); // lines with points on triangle tops/bottoms
		}
	}
	for (int k = 0; k<x_full.size(); k++) 
	{
		pts.push_back(x_full[k]); pts.push_back(SIZE_Y+AVG_EDGE_LENGTH); pts.push_back(0); // line between inside and boundary
	}
	for (int k = 0; k<x_full.size(); k++)
	{
		pts.push_back(x_full[k]); pts.push_back(SIZE_Y+2*AVG_EDGE_LENGTH); pts.push_back(0); // last line of points
	}

	// INDICES
	// boundary 
	std::vector<int> f_b0(6,0), f_b1(6,0);
	f_b0[0]=0; f_b0[1]=1; f_b0[2]=Nx+4; f_b0[3]=0; f_b0[4]=Nx+4; f_b0[5]=Nx+3; // line skip Nx+3	
	f_b1[0]=0; f_b1[1]=1; f_b1[2]=Nx+5; f_b1[3]=0; f_b1[4]=Nx+5; f_b1[5]=Nx+4; // line skip Nx+4
	
	for (int i = 0; i<Nx+2; i++) // stopping condition: i<=Nx+1
	{
		for (int k = 0; k<f_b0.size(); k++)
		{
			indices.push_back(f_b0[k]+i); // boundary bottom
		}
		for (int k = 0; k<f_b0.size(); k++)
		{
			indices.push_back(f_b0[k]+(Nx+3)*(1+Ny)+Ny/2+i); // boundary top
		}
	}

	for (int j = 0; j<Ny*(2*Nx+7)/2; j+=2*Nx+7) // stopping condition: j<=(Ny-2)*(2*Nx+7)/2
	{
		for (int k = 0; k<f_b0.size(); k++)
		{
			indices.push_back(f_b0[k]+Nx+3+j); // boundary left 
		}
		for (int k = 0; k<f_b1.size(); k++)
		{
			indices.push_back(f_b1[k]+2*Nx+4+j); // boundary right
		}
		for (int k = 0; k<f_b1.size(); k++)
		{
			indices.push_back(f_b1[k]+2*Nx+6+j); // boundary left
		}
		for (int k = 0; k<f_b0.size(); k++)
		{
			indices.push_back(f_b0[k]+3*Nx+8+j); // boundary right
		}
	}    
    
	// inside
	std::vector<int> f_down(3,0), f_up(3,0);
	f_down[0]=0; f_down[1]=1; f_down[2]=Nx+4; // triangles with base down
	f_up[0]=0; f_up[1]=Nx+4; f_up[2]=Nx+3; // triangles with base down

	for (int j = 0; j<(Ny/2)*(2*Nx+7); j+=2*Nx+7) // stopping condition: j<=(Ny/2-1)*(2*Nx+7)
	{
		for (int i = 0; i<Nx; i++) // stopping condition: i<=Nx-1
		{
			for (int k = 0; k<f_down.size(); k++)
			{
				indices.push_back(f_down[k]+Nx+4+i+j); 
			}
			for (int k = 0; k<f_up.size(); k++)
			{
				indices.push_back(f_up[k]+2*Nx+8+i+j);
			}
		}
		for (int i = 0; i<Nx+1; i++) // stopping condition: i<=Nx
		{
			for (int k = 0; k<f_up.size(); k++)
			{
				indices.push_back(f_up[k]+Nx+4+i+j); 
			}
			for (int k = 0; k<f_down.size(); k++)
			{
				indices.push_back(f_down[k]+2*Nx+7+i+j); 
			}
		}
	}
	
	std::vector<int> faces(indices.size()/3,3);

    mesh->build(pts.size()/3, &pts[0], faces.size(), &faces[0], &indices[0]);
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

double DeformableSimplicialComplex::intersection_with_link(const HMesh::VertexID& vid, CGLA::Vec2d destination) const
{
    double scale = INFINITY;
    CGLA::Vec2d p = get_pos(vid);
    CGLA::Vec2d r = destination - p;
    CGLA::Vec2d q, s;
    for(HMesh::Walker hew = walker(vid); !hew.full_circle(); hew = hew.circulate_vertex_cw())
    {
        q = get_pos(hew.vertex());
        s = get_pos(hew.next().vertex()) - q;
        double t = Util::intersection(p, r, q, s);
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
    CGLA::Vec2d v0_new = get_pos_new(vid);
    CGLA::Vec2d v0 = get_pos(vid);
    double l = (v0 - v0_new).length();
    if (l < EPSILON)
    {
        return true;
    }
    
    double scale = intersection_with_link(vid, v0_new);
    l = std::max(std::min(l*scale - MIN_DEFORMATION, l), 0.);
    CGLA::Vec2d v0_temp = v0 + l*normalize(v0_new - v0);
    
    CGLA::Vec2d v1, v2;
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

double DeformableSimplicialComplex::max_move_distance() const
{
    double dist, max_dist = 0.;
    for(HMesh::VertexIDIterator vi = vertices_begin(); vi != vertices_end(); ++vi)
    {
        if(is_movable(*vi))
        {
            dist = (new_pos[*vi] - get_pos(*vi)).length();
            max_dist = std::max(max_dist, dist);
        }
    }
    return max_dist;
}

std::vector<CGLA::Vec2d> DeformableSimplicialComplex::get_design_variable_positions()
{
    std::vector<CGLA::Vec2d> p;
    for (auto vi = vertices_begin(); vi != vertices_end(); vi++)
    {
        if(is_movable(*vi))
        {
            p.push_back(get_pos(*vi));
        }
    }
    return p;
}

std::vector<CGLA::Vec2d> DeformableSimplicialComplex::get_interface_edge_positions()
{
    std::vector<CGLA::Vec2d> p;
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

double DeformableSimplicialComplex::get_width() const
{
    return SIZE_X+2*BOUNDARY_GAP;
}

double DeformableSimplicialComplex::get_height() const
{
    return SIZE_Y+2*BOUNDARY_GAP;
}

CGLA::Vec2d DeformableSimplicialComplex::get_center()
{
    return design_domain->get_center();
}

CGLA::Vec2d DeformableSimplicialComplex::get_pos(HMesh::VertexID vid) const
{
    return CGLA::Vec2d(mesh->pos(vid)[0], mesh->pos(vid)[1]);
}

std::vector<CGLA::Vec2d> DeformableSimplicialComplex::get_pos(HMesh::FaceID fid) const
{
    std::vector<CGLA::Vec2d> positions;
    for (HMesh::Walker hew = walker(fid); !hew.full_circle(); hew = hew.circulate_face_cw())
    {
        positions.push_back( get_pos(hew.vertex()) );
    }
    return positions;
}

CGLA::Vec2d DeformableSimplicialComplex::get_pos_new(HMesh::VertexID vid) const
{
    return CGLA::Vec2d(new_pos[vid]);
}

std::vector<CGLA::Vec2d> DeformableSimplicialComplex::get_pos_new(HMesh::FaceID fid) const
{
    std::vector<CGLA::Vec2d> positions;
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
				labels[0] = std::min(get_label(hew.face()),	get_label(hew.opp().face()));
				labels[1] = std::max(get_label(hew.face()),	get_label(hew.opp().face()));				
			}
 		}
	}
	return labels;
}


void DeformableSimplicialComplex::set_pos(HMesh::VertexID vid, CGLA::Vec2d p)
{
    mesh->pos(vid) = CGLA::Vec3d(p[0], p[1], 0.f);
}

void DeformableSimplicialComplex::set_destination(HMesh::VertexID vid, CGLA::Vec2d dest)
{
    if(is_movable(vid))
    {
        CGLA::Vec2d vec = dest - get_pos(vid);
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
    std::vector<CGLA::Vec2d> positions;
    for(HMesh::Walker hew = walker(eid); !hew.full_circle(); hew = hew.circulate_vertex_ccw())
    {
        positions.push_back(get_pos(hew.vertex()));
    }
    
    double a, A;
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

double DeformableSimplicialComplex::length(HMesh::HalfEdgeID eid) const
{
    return HMesh::length(*mesh, eid);
}

double DeformableSimplicialComplex::length_new(HMesh::HalfEdgeID eid) const
{
    HMesh::Walker hew = walker(eid);
    return CGLA::length(get_pos_new(hew.vertex()) - get_pos_new(hew.opp().vertex()));
}

double DeformableSimplicialComplex::min_edge_length(HMesh::FaceID fid)
{
    std::vector<CGLA::Vec2d> p = get_pos(fid);
    return std::min(std::min(CGLA::length(p[0] - p[1]), CGLA::length(p[1] - p[2])), CGLA::length(p[0] - p[2]));
}

double DeformableSimplicialComplex::min_angle(HMesh::FaceID fid)
{
    std::vector<CGLA::Vec2d> p = get_pos(fid);
    return Util::min_angle(p[0], p[1], p[2]);
}

void DeformableSimplicialComplex::remove_needles()
{
    std::vector<HMesh::FaceID> fvec(get_no_faces());
    int i = 0;
    for(HMesh::FaceIDIterator fi = faces_begin(); fi != faces_end(); ++fi,++i)
    {
        fvec[i] = *fi;
    }
    
    std::vector<HMesh::HalfEdgeID> edges;
    for(i = 0; i < fvec.size(); ++i)
    {
        edges = sorted_face_edges(fvec[i]);
        if(min_angle(fvec[i]) < MIN_ANGLE
           && std::abs(length(edges[1]) - length(edges[0])) > std::abs(length(edges[2]) - length(edges[1])))
            //&& length(m,edges[2])/length(m,edges[1]) < 1.3
            //&& length(m,edges[2])/length(m,edges[1]) > 0.7)// length(m,edges[2])/length(m,edges[0]) > SPLIT_EDGE_RATIO)
        {
#ifdef DEBUG
            if (split(fvec[i]))
            {
                std::cout << "Split needle" << std::endl;
            }
#else
            split(fvec[i]);
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
                CGLA::Vec2d next = get_pos(hew.vertex()) - get_pos(hew.next().vertex());
                CGLA::Vec2d prev = get_pos(hew.prev().vertex()) - get_pos(hew.prev().prev().vertex());
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
        CGLA::Vec2d p0, p1, p2, p3;
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

CGLA::Vec2d DeformableSimplicialComplex::avg_pos(HMesh::VertexID vid, bool interface) const
{
    if(interface && !is_interface(vid))
    {
        return CGLA::Vec2d(0.f);
    }
    
    CGLA::Vec2d avg_pos(0.);
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
    assert(!CGLA::isnan(avg_pos[0]) && !CGLA::isnan(avg_pos[1]));
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

CGLA::Vec2d DeformableSimplicialComplex::filter_vertex(HMesh::VertexID vid, std::vector<double> &filter) const
{
	if(!is_interface(vid)&&!is_crossing(vid))
    {
        return CGLA::Vec2d(0.f);
    }
    CGLA::Vec2d sum_pos(0.);
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
	

CGLA::Vec2d DeformableSimplicialComplex::normal(HMesh::VertexID vid) const
{
    if(!is_interface(vid))
    {
        return CGLA::Vec2d(0.);
    }
    
    CGLA::Vec2d n(0.f), r(0.f);
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
            n += CGLA::Vec2d(r[1], -r[0]);
            i++;
        }
    }
#ifdef DEBUG
    assert(i == 2);
    assert(!CGLA::isnan(n[0]) && !CGLA::isnan(n[1]));
#endif
    if(sqr_length(n) > EPSILON)
    {
        n.normalize();
    }
    return n;
}

void DeformableSimplicialComplex::clamp_vector(const HMesh::VertexID& vid, CGLA::Vec2d& vec) const
{
    design_domain->clamp_vector(get_pos(vid), vec);
}

double DeformableSimplicialComplex::area(HMesh::FaceID fid) const
{
    return HMesh::area(*mesh, fid);
}

double DeformableSimplicialComplex::area_new(HMesh::FaceID fid) const
{
    std::vector<CGLA::Vec2d> positions = get_pos_new(fid);
    double A = std::abs(Util::signed_area<CGLA::Vec2d>(positions[0], positions[1], positions[2]));
#ifdef DEBUG
    assert(A > 0.);
#endif
    return A;
}

void DeformableSimplicialComplex::smooth(double t)
{
    std::vector<CGLA::Vec2d> positions(get_no_vertices());
    int i = 0;
    for(HMesh::VertexIDIterator vi = vertices_begin(); vi != vertices_end(); ++vi,++i)
    {
        if(safe_editable(*vi))
        {
			positions[i] =  t * (avg_pos(*vi, false) - get_pos(*vi)) + get_pos(*vi);
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
                double angle = Util::cos_angle(get_pos(vertices[0]), get_pos(vid), get_pos(vertices[1]));
                //                double angle = dot(normalize(get_pos(vertices[0]) - get_pos(vid)), normalize(get_pos(vertices[1]) - get_pos(vid)));
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
    
    double energy = efun.delta_energy(*mesh, h);
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