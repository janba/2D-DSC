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

#ifndef ___D_DSC__draw__
#define ___D_DSC__draw__

#include "DSC.h"
#include "velocity_function.h"

const static double POINT_SIZE = 0.2;
const static double LINE_WIDTH = 0.1;

const static CGLA::Vec3d BACKGROUND_COLOR = CGLA::Vec3d(0.7);
const static CGLA::Vec3d INVISIBLE = CGLA::Vec3d(-1.);
const static CGLA::Vec3d DARK_RED = CGLA::Vec3d(0.66,0.11,0.15);
const static CGLA::Vec3d RED = CGLA::Vec3d(0.96,0.11,0.15);
const static CGLA::Vec3d DARK_BLUE = CGLA::Vec3d(0.14,0.16,0.88);
const static CGLA::Vec3d BLUE = CGLA::Vec3d(0.45,0.7,0.9);
const static CGLA::Vec3d GREEN = CGLA::Vec3d(0.05,1.,0.15);
const static CGLA::Vec3d ORANGE = CGLA::Vec3d(0.9,0.4,0.);
const static CGLA::Vec3d BLACK = CGLA::Vec3d(0.);
const static CGLA::Vec3d DARK_GRAY = CGLA::Vec3d(0.5);
const static CGLA::Vec3d GRAY = CGLA::Vec3d(0.8);

/**
 A painter handles all draw functionality using OpenGL.
 */
class Painter {
    
    
public:
    /**
     Saves the current painting to the selected folder.
     */
    static void save_painting(int width, int height, std::string folder = std::string(""), int time_step = -1);
    
    /**
     Begins drawing.
     */
    static void begin();
    
    /**
     Ends drawing.
     */
    static void end();
    
    /**
     Draws the simplicial complex.
     */
    static void draw_complex(const DeformableSimplicialComplex& complex);
    
    /**
     Draws the domain.
     */
    static void draw_domain(const Domain& domain, CGLA::Vec3d color = GRAY);
    
    /**
     Draws the vertices with the colors defined by the get_vertex_colors function in the simplicial complex.
     */
    static void draw_vertices(const DeformableSimplicialComplex& complex);

    /**
     Draws the edges with the colors defined by the get_edge_colors function in the simplicial complex.
     */
    static void draw_edges(const DeformableSimplicialComplex& complex);
    
    /**
     Draws the faces with the colors defined by the get_face_colors function in the simplicial complex.
     */
    static void draw_faces(const DeformableSimplicialComplex& complex);
    
    /**
     Draws the faces with the colors given as input.
     */
    static void draw_faces(const DeformableSimplicialComplex& complex, const HMesh::FaceAttributeVector<CGLA::Vec3d> &colors);
    
    /**
     Draws the faces using the 'jet' color scheme.
     */
    static void draw_faces(const DeformableSimplicialComplex& complex, const HMesh::FaceAttributeVector<double> &values);
    
    /**
     Draws the interface with the color given as input.
     */
    static void draw_interface(const DeformableSimplicialComplex& complex, CGLA::Vec3d color = ORANGE);
    
    /**
     Draws the arrows given as input with the color given as input.
     */
    static void draw_arrows(const DeformableSimplicialComplex& complex, const HMesh::VertexAttributeVector<CGLA::Vec2d> &arrows, CGLA::Vec3d color = ORANGE);

    /**
     Draws the lines given as input with the color given as input.
     */
    static void draw_lines(const DeformableSimplicialComplex& complex, const HMesh::VertexAttributeVector<CGLA::Vec2d> &lines, CGLA::Vec3d color = GREEN);
};


#endif /* defined(___D_DSC__draw__) */
