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

#include "draw.h"

#ifdef WIN32
#include <GL/glew.h>
#include <GL/glut.h>
#include <GLGraphics/SOIL.h>
#else
#include <GEL/GL/glew.h>
#include <GLUT/glut.h>
#include <GEL/GLGraphics/SOIL.h>
#endif


void Painter::save_painting(int width, int height, std::string folder, int time_step)
{
    std::ostringstream s;
    if (folder.length() == 0) {
        s << "scr";
    }
    else {
        s << folder << "/scr";
    }
    
    if (time_step >= 0)
    {
        s << std::string(Util::concat4digits("_", time_step));
    }
    s << ".png";
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    int success = SOIL_save_screenshot(s.str().c_str(), SOIL_SAVE_TYPE_PNG, 0, 0, width, height);
    if(!success)
    {
        std::cout << "ERROR: Failed to take screen shot: " << s.str().c_str() << std::endl;
        return;
    }
}


void Painter::begin()
{
    glClearColor(BACKGROUND_COLOR[0], BACKGROUND_COLOR[1], BACKGROUND_COLOR[2],0);
    glClear(GL_COLOR_BUFFER_BIT);
}

void Painter::end()
{
    glFinish();
    glutSwapBuffers();
}


void Painter::draw_complex(const DeformableSimplicialComplex& dsc)
{
    draw_domain(*dsc.get_design_domain());
    std::vector<Domain*> objects = dsc.get_object_domains();
//    for (auto obj = objects.begin(); obj != objects.end(); obj++)
//    {
//        draw_domain(*obj, DARK_GRAY);
//    }
    draw_faces(dsc);
    draw_edges(dsc);
    draw_vertices(dsc);
}

void Painter::draw_domain(const Domain& domain, CGLA::Vec3d color)
{
    std::vector<CGLA::Vec2d> corners = domain.get_corners();
    glColor3dv(&color[0]);
    CGLA::Vec2d p0, p1, p2;
    CGLA::Vec3d cor;
    int j = 0, i;
    glBegin(GL_TRIANGLES);
    while (corners.size() > 2)
    {
        i = (j+1)%corners.size();
        p0 = corners[j%corners.size()];
        p1 = corners[i];
        p2 = corners[(j+2)%corners.size()];
        if (!Util::is_left_of(p0, p1, p2))
        {
            cor = CGLA::Vec3d(p0[0], p0[1], 0.);
            glVertex3dv(&cor[0]);
            cor = CGLA::Vec3d(p1[0], p1[1], 0.);
            glVertex3dv(&cor[0]);
            cor = CGLA::Vec3d(p2[0], p2[1], 0.);
            glVertex3dv(&cor[0]);
            corners.erase(corners.begin() + i);
        }
        else
        {
            j = i;
        }
    }
    glEnd();
}

void Painter::draw_vertices(const DeformableSimplicialComplex& dsc)
{
    HMesh::VertexAttributeVector<CGLA::Vec3d> colors = dsc.get_vertex_colors();
    glPointSize(std::max(std::floor(POINT_SIZE*dsc.get_avg_edge_length()), 1.));
	glBegin(GL_POINTS);
    CGLA::Vec3d p;
	for(HMesh::VertexIDIterator vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        p = CGLA::Vec3d(dsc.get_pos(*vi)[0], dsc.get_pos(*vi)[1], 0.);
        glColor3dv(&colors[*vi][0]);
        glVertex3dv(&p[0]);
    }
	glEnd();
}

void Painter::draw_interface(const DeformableSimplicialComplex& dsc, CGLA::Vec3d color)
{
    glPointSize(std::max(std::floor(POINT_SIZE*dsc.get_avg_edge_length()), 1.));
	glBegin(GL_POINTS);
    CGLA::Vec3d p;
    glColor3dv(&color[0]);
	for(HMesh::VertexIDIterator vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if (dsc.is_movable(*vi)) {
            CGLA::Vec3d temp(dsc.get_pos_new(*vi)[0], dsc.get_pos_new(*vi)[1], 0.);
            glVertex3dv(&temp[0]);
        }
    }
	glEnd();
    glLineWidth(std::max(std::floor(LINE_WIDTH*dsc.get_avg_edge_length()), 1.));
    CGLA::Vec3d p1, p2;
	glBegin(GL_LINES);
    for(HMesh::HalfEdgeIDIterator hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei)
    {
        HMesh::Walker hew = dsc.walker(*hei);
        if (dsc.is_movable(hew.halfedge()) && (dsc.is_movable(hew.vertex()) || dsc.is_movable(hew.opp().vertex())))
        {
            p1 = CGLA::Vec3d(dsc.get_pos_new(hew.vertex())[0], dsc.get_pos_new(hew.vertex())[1], 0.);
            p2 = CGLA::Vec3d(dsc.get_pos_new(hew.opp().vertex())[0], dsc.get_pos_new(hew.opp().vertex())[1], 0.);
            glVertex3dv(&p1[0]);
            glVertex3dv(&p2[0]);
        }
    }
	glEnd();
}

void Painter::draw_arrows(const DeformableSimplicialComplex& dsc, const HMesh::VertexAttributeVector<CGLA::Vec2d> &arrows, CGLA::Vec3d color)
{
    glColor3dv(&color[0]);
    glLineWidth(std::max(std::floor(LINE_WIDTH*dsc.get_avg_edge_length()), 1.));
    CGLA::Vec3d arrow, a_hat, p;
    for(HMesh::VertexIDIterator vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        arrow = CGLA::Vec3d(arrows[*vi][0], arrows[*vi][1], 0.f);
        if(arrow.length() > EPSILON)
        {
            a_hat = CGLA::Vec3d(-arrow[1], arrow[0], 0.f);
            p = CGLA::Vec3d(dsc.get_pos(*vi)[0], dsc.get_pos(*vi)[1], 0.);
#ifdef DEBUG
            if (dsc.is_movable(*vi)) {
                p = CGLA::Vec3d(dsc.get_pos_new(*vi)[0], dsc.get_pos_new(*vi)[1], 0.);
            }
#endif
            glBegin(GL_LINES);
            glVertex3dv(&p[0]);
            glVertex3dv(&(p + 0.7*arrow)[0]);
            glEnd();
            
            glBegin(GL_POLYGON);
            glVertex3dv(&(p + arrow)[0]);
            glVertex3dv(&(p + 0.6*arrow + 0.13*a_hat)[0]);
            glVertex3dv(&(p + 0.6*arrow - 0.13*a_hat)[0]);
            glEnd();
        }
    }
}


void Painter::draw_lines(const DeformableSimplicialComplex& dsc, const HMesh::VertexAttributeVector<CGLA::Vec2d> &lines, CGLA::Vec3d color)
{
    glColor3dv(&color[0]);
    glLineWidth(std::max(std::floor(LINE_WIDTH*dsc.get_avg_edge_length()), 1.));
    CGLA::Vec3d line, p;
    for(HMesh::VertexIDIterator vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        line = CGLA::Vec3d(lines[*vi][0], lines[*vi][1], 0.f);
        if(line.length() > EPSILON)
        {
            p = CGLA::Vec3d(dsc.get_pos(*vi)[0], dsc.get_pos(*vi)[1], 0.);
            
            glBegin(GL_LINES);
            glVertex3dv(&p[0]);
            glVertex3dv(&(p + line)[0]);
            glEnd();
        }
    }
}

void Painter::draw_edges(const DeformableSimplicialComplex& dsc)
{
    HMesh::HalfEdgeAttributeVector<CGLA::Vec3d> colors = dsc.get_edge_colors();
    glLineWidth(std::max(std::floor(LINE_WIDTH*dsc.get_avg_edge_length()), 1.));
    CGLA::Vec3d p1, p2;
	glBegin(GL_LINES);
	for(HMesh::HalfEdgeIDIterator hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei)
    {
        glColor3dv(&colors[*hei][0]);
        
        HMesh::Walker hew = dsc.walker(*hei);
        p1 = CGLA::Vec3d(dsc.get_pos(hew.vertex())[0], dsc.get_pos(hew.vertex())[1], 0.);
        p2 = CGLA::Vec3d(dsc.get_pos(hew.opp().vertex())[0], dsc.get_pos(hew.opp().vertex())[1], 0.);
        glVertex3dv(&p1[0]);
        glVertex3dv(&p2[0]);
    }
	glEnd();
}

void Painter::draw_faces(const DeformableSimplicialComplex& dsc)
{
    HMesh::FaceAttributeVector<CGLA::Vec3d> colors = dsc.get_face_colors();
    draw_faces(dsc, colors);
}

void Painter::draw_faces(const DeformableSimplicialComplex& dsc, const HMesh::FaceAttributeVector<CGLA::Vec3d> &colors)
{
    glBegin(GL_TRIANGLES);
	for(HMesh::FaceIDIterator fi = dsc.faces_begin(); fi != dsc.faces_end(); ++fi)
    {
        if(colors[*fi] != INVISIBLE)
        {
            glColor3dv(&colors[*fi][0]);
            for (HMesh::Walker hew = dsc.walker(*fi); !hew.full_circle(); hew = hew.circulate_face_cw())
            {
                glVertex3dv(&dsc.get_pos(hew.vertex())[0]);
            }
        }
    }
    glEnd();
}

void Painter::draw_faces(const DeformableSimplicialComplex& dsc, const HMesh::FaceAttributeVector<double> &values)
{
    glBegin(GL_TRIANGLES);
	for(HMesh::FaceIDIterator fi = dsc.faces_begin(); fi != dsc.faces_end(); ++fi)
    {
        if(values[*fi] >= 0.)
        {
            CGLA::Vec3d col = Util::jet_color(values[*fi]);
            glColor3dv(&col[0]);
            for (HMesh::Walker hew = dsc.walker(*fi); !hew.full_circle(); hew = hew.circulate_face_cw())
            {
                glVertex3dv(&dsc.get_pos(hew.vertex())[0]);
            }
        }
    }
    glEnd();
}

