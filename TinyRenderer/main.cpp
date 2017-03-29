#include <vector>
#include <cmath>
#include <stdlib.h>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

using namespace std;

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green   = TGAColor(0, 255,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1,TGAImage &image, const TGAColor c){
  bool steep = false; 
  if (std::abs(x0-x1)<std::abs(y0-y1)) { //Si la ligne est plus grande en vertical on la parcourt en vertical 
    std::swap(x0, y0); 
    std::swap(x1, y1); 
    steep = true; 
  } 
  if (x0>x1) { // faire en sorte de toujours commencer par la "gauche" 
    std::swap(x0, x1); 
    std::swap(y0, y1); 
  } 
  int dx = x1 - x0;
  int dy = y1 - y0;
  
  int derror2 = std::abs(dy)*2; //on supprime les floats en faisant *2 pour supprimmer la comparaison avec 0.5 et on fait *dx pour enlever la division
  int error2 = 0; 
  int y = y0; 
  for (int x=x0; x<=x1; x++) { 
    if (steep) { 
      image.set(y, x, c); 
    } else { 
      image.set(x, y, c); 
    } 
    error2 += derror2; 
    if (error2 > dx) { 
      y += (y1>y0?1:-1); 
      error2 -= dx*2; 
    } 
  } 
} 

void line(Vec2i l1,Vec2i l2,TGAImage &image, const TGAColor c){
  bool steep = false;
  int x0 = l1.x;
  int x1 = l2.x;
  int y0 = l1.y;
  int y1 = l2.y;
  if (std::abs(x0-x1)<std::abs(y0-y1)) { //Si la ligne est plus grande en vertical on la parcourt en vertical
    std::swap(x0, y0);
    std::swap(x1, y1);
    steep = true;
  }
  if (x0>x1) { // faire en sorte de toujours commencer par la "gauche"
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int dx = x1 - x0;
  int dy = y1 - y0;

  int derror2 = std::abs(dy)*2; //on supprime les floats en faisant *2 pour supprimmer la comparaison avec 0.5 et on fait *dx pour enlever la division
  int error2 = 0;
  int y = y0;
  for (int x=x0; x<=x1; x++) {
    if (steep) {
      image.set(y, x, c);
    } else {
      image.set(x, y, c);
    }
    error2 += derror2;
    if (error2 > dx) {
      y += (y1>y0?1:-1);
      error2 -= dx*2;
    }
  }
}

Vec3f barycentric(Vec2i *p, Vec2i P) {
    //Sur un triangle ABC

    //D = (Ax-Cx)(By-Cy) - ((Bx-Cx)(Ay-Cy))
    double d = ((p[0].x-p[2].x)*(p[1].y-p[2].y))-((p[1].x-p[2].x)*(p[0].y-p[2].y));
    //Alpha
    double x = (1/d)*((p[1].y-p[2].y)*(P.x-p[2].x) + (p[2].x-p[1].x)*(P.y-p[2].y));
    //Beta
    double y = (1/d)*((p[2].y-p[0].y)*(P.x-p[2].x) + (p[0].x-p[2].x)*(P.y-p[2].y));
    //gamma = 1-alpha - beta

    if (abs(x+y)>1)
           return Vec3f(-1,1,1);
    return Vec3f(x,y,1-x-y);
}

void triangle(Vec2i *pts, TGAImage &image, TGAColor color) {
    //calcul de la bounding box
   Vec2i bboxmin,bboxmax;
   bboxmin.x = min(pts[0].x,min(pts[1].x,pts[2].x));
   bboxmax.x = max(pts[0].x,max(pts[1].x,pts[2].x));
   bboxmin.y = min(pts[0].y,min(pts[1].y,pts[2].y));
   bboxmax.y = max(pts[0].y,max(pts[1].y,pts[2].y));

    Vec2i P;
    //dans la bounding box on vérifie les points qui sont dans le triangle
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f barycentre  = barycentric(pts, P);
             if (barycentre.x>=0 && barycentre.y>=0 && barycentre.z>=0)
            image.set(P.x, P.y, color);
        }
    }
}

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

   TGAImage image(width, height, TGAImage::RGB);
   Vec3f light_dir(0,0,-1);
   for (int i=0; i<model->nfaces(); i++) {
       std::vector<int> face = model->face(i);
       Vec2i screen_coords[3];
       Vec3f world_coords[3];

       for (int j=0; j<3; j++) {
           Vec3f v = model->vert(face[j]);
           screen_coords[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.);
           world_coords[j]  = v;
       }
       Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
       n.normalize();
       float intensity = n*light_dir;
       if (intensity>0) {
           triangle(screen_coords, image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
       }
   }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

