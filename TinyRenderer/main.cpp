#include <vector>
#include <cmath>
#include <limits>
#include <stdlib.h>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

using namespace std;

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green   = TGAColor(0, 255,   0,   255);
const TGAColor blue   = TGAColor(0, 0,   255,   255);
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
    float d = ((p[0].x-p[2].x)*(p[1].y-p[2].y))-((p[1].x-p[2].x)*(p[0].y-p[2].y));
    //Alpha
    float x = (1/d)*((p[1].y-p[2].y)*(P.x-p[2].x) + (p[2].x-p[1].x)*(P.y-p[2].y));
    //Beta
    float y = (1/d)*((p[2].y-p[0].y)*(P.x-p[2].x) + (p[0].x-p[2].x)*(P.y-p[2].y));
    //gamma = 1-alpha - beta

    if (abs(x+y)>1)
           return Vec3f(-1,1,1);
    return Vec3f(x,y,1-x-y);
}

Vec3f barycentric(Vec3f *p, Vec3f P) {
    //Sur un triangle ABC

    //D = (Ax-Cx)(By-Cy) - ((Bx-Cx)(Ay-Cy))
    float d = ((p[0].x-p[2].x)*(p[1].y-p[2].y))-((p[1].x-p[2].x)*(p[0].y-p[2].y));
    //Alpha
    float x = (1/d)*((p[1].y-p[2].y)*(P.x-p[2].x) + (p[2].x-p[1].x)*(P.y-p[2].y));
    //Beta
    float y = (1/d)*((p[2].y-p[0].y)*(P.x-p[2].x) + (p[0].x-p[2].x)*(P.y-p[2].y));
    //gamma = 1-alpha - beta

    if (abs(x+y)>1)
           return Vec3f(-1,1,1);
    return Vec3f(x,y,1-x-y);
}


/*void triangle(Vec2i *pts, TGAImage &image, TGAColor color) {
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
}*/

void triangle(Vec3f *pts,Vec2f u0, Vec2f u1, Vec2f u2,Vec3f v0, Vec3f v1, Vec3f v2,float zbuffer[], TGAImage &image,Vec3f light_dir) {
    //calcul de la bounding box
   Vec2f bboxmin,bboxmax;

   bboxmin.x = min(pts[0].x,min(pts[1].x,pts[2].x));
   bboxmax.x = max(pts[0].x,max(pts[1].x,pts[2].x));
   bboxmin.y = min(pts[0].y,min(pts[1].y,pts[2].y));
   bboxmax.y = max(pts[0].y,max(pts[1].y,pts[2].y));

   Vec3f P;
   Vec2f up;
   Vec3f n,np;
   for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
       for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f barycentre  = barycentric(pts, P);
            if (barycentre.x>=-1e-2 && barycentre.y>=-1e-2 && barycentre.z>=-1e-2){
                P.z = 0;
                P.z += pts[0].z*barycentre.x;
                P.z += pts[1].z*barycentre.y;
                P.z += pts[2].z*barycentre.z;

                up = u0 * barycentre.x +  u1 * barycentre.y +u2 * barycentre.z ;


                np = v0 * barycentre.x +  v1 * barycentre.y +v2 * barycentre.z ;
                np.normalize();
                if (zbuffer[int(P.x+P.y*width)]<P.z) {
                    zbuffer[int(P.x+P.y*width)] = P.z;
                    //calcul de la couleur

                    float intensity = -(np*light_dir);
                    TGAColor color = model->diffuse(up);
                    image.set(P.x, P.y, color*(intensity));
                }
            }
        }
   }

}

void rasterize(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color, int ybuffer[]) {
    if (p0.x>p1.x)
        std::swap(p0, p1);
    for(int i = p0.x;i < p1.x;i++){
        //calcul de y comme dans la premiere partie du tuto
        float t = (i-p0.x)/(float)(p1.x-p0.x);
        int y = p0.y * (1-t) + p1.y*t;

        //on ne dessine que quand la valeur change (possible car le tableau est initialisé a -infini
        if(y > ybuffer[i]){
            ybuffer[i] = y;
            image.set(i, 0, color);
        }
    }
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z);
}

void transform(Vec3f *v,float c){
    for(int i = 0;i< 3;i++){
        cout << i<<endl;
        v[i][0] = v[i][0]/((1-v[i][2])/c);
        v[i][1] = v[i][1]/((1-v[i][2])/c);
        v[i][2] = v[i][2]/((1-v[i][2])/c);
    }
}

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

    float *zbuffer = new float[width*height];
    Vec3f light_dir(0,0,-1);

    for (int i=width*height; i--; zbuffer[i] = -std::numeric_limits<float>::max());

    Vec2f u0,u1,u2;
    Vec3f v0,v1,v2;

    TGAImage image(width, height, TGAImage::RGB);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f pts_s[3];
        for (int i=0; i<3; i++){
            pts_s[i] = world2screen(model->vert(face[i]));
        }
         //   transform(pts_s,2);
            v0 = model->normal(i,0);
            v1 = model->normal(i,1);
            v2 = model->normal(i,2);
            u0 = model->uv(i,0);
            u1 = model->uv(i,1);
            u2 = model->uv(i,2);
            triangle(pts_s,u0,u1,u2,v0,v1,v2,zbuffer, image,light_dir);
    }

    image.flip_vertically();
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

