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

using namespace DSC2D;

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
    draw_faces(dsc);
    draw_edges(dsc);
    draw_vertices(dsc);
}

void Painter::draw_domain(const DesignDomain& domain, vec3 color)
{
    std::vector<vec2> corners = domain.get_corners();
    glColor3d(static_cast<double>(color[0]), static_cast<double>(color[1]), static_cast<double>(color[2]));
    vec2 p0, p1, p2;
    vec3 cor;
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
            cor = vec3(p0[0], p0[1], 0.);
            glVertex3d(static_cast<double>(cor[0]), static_cast<double>(cor[1]), static_cast<double>(cor[2]));
            cor = vec3(p1[0], p1[1], 0.);
            glVertex3d(static_cast<double>(cor[0]), static_cast<double>(cor[1]), static_cast<double>(cor[2]));
            cor = vec3(p2[0], p2[1], 0.);
            glVertex3d(static_cast<double>(cor[0]), static_cast<double>(cor[1]), static_cast<double>(cor[2]));
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
    HMesh::VertexAttributeVector<vec3> colors = dsc.get_vertex_colors();
    glPointSize(std::max(std::floor(POINT_SIZE*dsc.get_avg_edge_length()), 1.));
	glBegin(GL_POINTS);
    vec3 p;
	for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        p = vec3(dsc.get_pos(*vi)[0], dsc.get_pos(*vi)[1], 0.);
        glColor3d(static_cast<double>(colors[*vi][0]), static_cast<double>(colors[*vi][1]), static_cast<double>(colors[*vi][2]));
        glVertex3d(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]));
    }
	glEnd();
}

void Painter::draw_interface(const DeformableSimplicialComplex& dsc, vec3 color)
{
    glPointSize(std::max(std::floor(POINT_SIZE*dsc.get_avg_edge_length()), 1.));
	glBegin(GL_POINTS);
    vec3 p;
    glColor3d(static_cast<double>(color[0]), static_cast<double>(color[1]), static_cast<double>(color[2]));
	for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        if (dsc.is_movable(*vi)) {
            vec3 temp(dsc.get_destination(*vi)[0], dsc.get_destination(*vi)[1], 0.);
            glVertex3d(static_cast<double>(temp[0]), static_cast<double>(temp[1]), static_cast<double>(temp[2]));
        }
    }
	glEnd();
    glLineWidth(std::max(std::floor(LINE_WIDTH*dsc.get_avg_edge_length()), 1.));
    vec3 p1, p2;
	glBegin(GL_LINES);
    for(auto hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei)
    {
        auto hew = dsc.walker(*hei);
        if (dsc.is_movable(hew.halfedge()) && (dsc.is_movable(hew.vertex()) || dsc.is_movable(hew.opp().vertex())))
        {
            p1 = vec3(dsc.get_destination(hew.vertex())[0], dsc.get_destination(hew.vertex())[1], 0.);
            p2 = vec3(dsc.get_destination(hew.opp().vertex())[0], dsc.get_destination(hew.opp().vertex())[1], 0.);
            glVertex3d(static_cast<double>(p1[0]), static_cast<double>(p1[1]), static_cast<double>(p1[2]));
            glVertex3d(static_cast<double>(p2[0]), static_cast<double>(p2[1]), static_cast<double>(p2[2]));
        }
    }
	glEnd();
}

void Painter::draw_arrows(const DeformableSimplicialComplex& dsc, const HMesh::VertexAttributeVector<vec2> &arrows, vec3 color)
{
    glColor3d(static_cast<double>(color[0]), static_cast<double>(color[1]), static_cast<double>(color[2]));
    glLineWidth(std::max(std::floor(LINE_WIDTH*dsc.get_avg_edge_length()), 1.));
    vec3 arrow, a_hat, p;
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        arrow = vec3(arrows[*vi][0], arrows[*vi][1], 0.f);
        if(arrow.length() > EPSILON)
        {
            a_hat = vec3(-arrow[1], arrow[0], 0.f);
            p = vec3(dsc.get_pos(*vi)[0], dsc.get_pos(*vi)[1], 0.);
#ifdef DEBUG
            if (dsc.is_movable(*vi)) {
                p = vec3(dsc.get_destination(*vi)[0], dsc.get_destination(*vi)[1], 0.);
            }
#endif
            glBegin(GL_LINES);
            glVertex3d(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]));
            glVertex3d(static_cast<double>((p + 0.7*arrow)[0]), static_cast<double>((p + 0.7*arrow)[1]), static_cast<double>((p + 0.7*arrow)[2]));
            glEnd();
            
            glBegin(GL_POLYGON);
            glVertex3d(static_cast<double>((p + arrow)[0]), static_cast<double>((p + arrow)[1]), static_cast<double>((p + arrow)[2]));
            glVertex3d(static_cast<double>((p + 0.6*arrow + 0.13*a_hat)[0]), static_cast<double>((p+ 0.6*arrow + 0.13*a_hat)[1]), static_cast<double>((p+ 0.6*arrow + 0.13*a_hat)[2]));
            glVertex3d(static_cast<double>((p + 0.6*arrow - 0.13*a_hat)[0]), static_cast<double>((p + 0.6*arrow - 0.13*a_hat)[1]), static_cast<double>((p + 0.6*arrow - 0.13*a_hat)[2]));
            glEnd();
        }
    }
}


void Painter::draw_lines(const DeformableSimplicialComplex& dsc, const HMesh::VertexAttributeVector<vec2> &lines, vec3 color)
{
    glColor3d(static_cast<double>(color[0]), static_cast<double>(color[1]), static_cast<double>(color[2]));
    glLineWidth(std::max(std::floor(LINE_WIDTH*dsc.get_avg_edge_length()), 1.));
    vec3 line, p;
    for(auto vi = dsc.vertices_begin(); vi != dsc.vertices_end(); ++vi)
    {
        line = vec3(lines[*vi][0], lines[*vi][1], 0.f);
        if(line.length() > EPSILON)
        {
            p = vec3(dsc.get_pos(*vi)[0], dsc.get_pos(*vi)[1], 0.);
            
            glBegin(GL_LINES);
            glVertex3d(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]));
            glVertex3d(static_cast<double>((p + line)[0]), static_cast<double>((p + line)[1]), static_cast<double>((p + line)[2]));
            glEnd();
        }
    }
}

void Painter::draw_edges(const DeformableSimplicialComplex& dsc)
{
    HMesh::HalfEdgeAttributeVector<vec3> colors = dsc.get_edge_colors();
    glLineWidth(std::max(std::floor(LINE_WIDTH*dsc.get_avg_edge_length()), 1.));
    vec3 p1, p2;
	glBegin(GL_LINES);
	for(auto hei = dsc.halfedges_begin(); hei != dsc.halfedges_end(); ++hei)
    {
        glColor3d(static_cast<double>(colors[*hei][0]), static_cast<double>(colors[*hei][1]), static_cast<double>(colors[*hei][2]));
        
        auto hew = dsc.walker(*hei);
        p1 = vec3(dsc.get_pos(hew.vertex())[0], dsc.get_pos(hew.vertex())[1], 0.);
        p2 = vec3(dsc.get_pos(hew.opp().vertex())[0], dsc.get_pos(hew.opp().vertex())[1], 0.);
        glVertex3d(static_cast<double>(p1[0]), static_cast<double>(p1[1]), static_cast<double>(p1[2]));
        glVertex3d(static_cast<double>(p2[0]), static_cast<double>(p2[1]), static_cast<double>(p2[2]));
    }
	glEnd();
}

void Painter::draw_faces(const DeformableSimplicialComplex& dsc)
{
    HMesh::FaceAttributeVector<vec3> colors = dsc.get_face_colors();
    draw_faces(dsc, colors);
}

void Painter::draw_faces(const DeformableSimplicialComplex& dsc, const HMesh::FaceAttributeVector<vec3> &colors)
{
    glBegin(GL_TRIANGLES);
	for(auto fi = dsc.faces_begin(); fi != dsc.faces_end(); ++fi)
    {
        if(colors[*fi] != INVISIBLE)
        {
            glColor3d(static_cast<double>(colors[*fi][0]), static_cast<double>(colors[*fi][1]), static_cast<double>(colors[*fi][2]));
            for (auto hew = dsc.walker(*fi); !hew.full_circle(); hew = hew.circulate_face_cw())
            {
                vec2 p = dsc.get_pos(hew.vertex());
                glVertex3d(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(0.));
            }
        }
    }
    glEnd();
}

void Painter::draw_faces(const DeformableSimplicialComplex& dsc, const HMesh::FaceAttributeVector<double> &values)
{
    glBegin(GL_TRIANGLES);
	for(auto fi = dsc.faces_begin(); fi != dsc.faces_end(); ++fi)
    {
        if(values[*fi] >= 0.)
        {
            vec3 color = Util::jet_color(values[*fi]);
            glColor3d(static_cast<double>(color[0]), static_cast<double>(color[1]), static_cast<double>(color[2]));
            for (auto hew = dsc.walker(*fi); !hew.full_circle(); hew = hew.circulate_face_cw())
            {
                vec2 p = dsc.get_pos(hew.vertex());
                glVertex3d(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(0.));
            }
        }
    }
    glEnd();
}

