#include "rasterizer.h"

using namespace std;

namespace CGL {
  RasterizerImp::RasterizerImp(PixelSampleMethod psm, LevelSampleMethod lsm,
    size_t width, size_t height,
    unsigned int sample_rate) {
    this->psm = psm;
    this->lsm = lsm;
    this->width = width;
    this->height = height;
    this->sample_rate = sample_rate;

    sample_buffer.resize(width * height * sample_rate, Color::White);
  }



  // Used by rasterize_point and rasterize_line
  void RasterizerImp::fill_pixel(size_t x, size_t y, Color c) {
    // TODO: Task 2: You might need to this function to fix points and lines (such as the black rectangle border in test4.svg)
    // NOTE: You are not required to implement proper supersampling for points and lines
    // It is sufficient to use the same color for all supersamples of a pixel for points and lines (not triangles)

    // y: how many pixels before the row
    // x: how many pixels before the col
    // sampling -> filling in sample array
    // printf("ss %d, %ld\n", x, y);
    // float eff_x = x * sqrt(this->sample_rate), eff_y = y * sqrt(this->sample_rate), eff_wid = width * sqrt(this->sample_rate);
    // printf("col %f %f %f\n", c.r, c.g, c.b);
    // if (c.r == 1.0 && c.g == 1.0 && c.b == 1.0) printf("caught %ld %ld! \n", x, y);
    sample_buffer[y * width * sqrt(this->sample_rate) + x] = c;
  }

  // Rasterize a point: simple example to help you start familiarizing
  // yourself with the starter code.
  //
  void RasterizerImp::rasterize_point(float x, float y, Color color) {
    // fill in the nearest pixel
    int sx = round(floor(x)) * sqrt(this->sample_rate);
    int sy = round(floor(y)) * sqrt(this->sample_rate);
    // check bounds
    if (sx < 0 || sx >= width * sqrt(this->sample_rate)) return;
    if (sy < 0 || sy >= height * sqrt(this->sample_rate)) return;


    for (size_t i = 0; i < sqrt(this->sample_rate); i++) {
      for (size_t j = 0; j < sqrt(this->sample_rate); j++) {
        fill_pixel(sx+i, sy+j, color);
      }
    }
    
    return;
  }

  // Rasterize a line.
  void RasterizerImp::rasterize_line(float x0, float y0,
    float x1, float y1,
    Color color) {
    if (x0 > x1) {
      swap(x0, x1); swap(y0, y1);
    }

    float pt[] = { x0,y0 };
    float m = (y1 - y0) / (x1 - x0);
    float dpt[] = { 1,m };
    int steep = abs(m) > 1;
    if (steep) {
      dpt[0] = x1 == x0 ? 0 : 1 / abs(m);
      dpt[1] = x1 == x0 ? (y1 - y0) / abs(y1 - y0) : m / abs(m);
    }
    while (floor(pt[0]) <= floor(x1) && abs(pt[1] - y0) <= abs(y1 - y0)) {
      // rasterize the nearest pixel for a given pt
      rasterize_point(pt[0], pt[1], color);
      pt[0] += dpt[0]; pt[1] += dpt[1];
    }
  }

  // Rasterize a triangle.
  void RasterizerImp::rasterize_triangle(float x0, float y0,
    float x1, float y1,
    float x2, float y2,
    Color color) {
    // TODO: Task 1: Implement basic triangle rasterization here, no supersampling
    // printf("invkked\n");

    // printf("bbox has %d %d %d %d\n", bbox[0], bbox[1], bbox[2], bbox[3]);

    // tri: {xa, ya, xb, yb, xc, yc}
    // pt: {xp, yp}
    // pt_bary: {alphaA, betaB, lamdaC}
    // init original pt and bary pt vector and triangle
    // bbox with edge vertices (bbox[0], bbox[1]), (bbox[2], bbox[3])

    double pix_siz= 1.0/sqrt(this->sample_rate);
    Vector2D a(x0,y0), b(x1,y1), c(x2,y2), pt; 
    Vector2D bbox_ll(floor(threeMin(x0, x1, x2)), floor(threeMin(y0, y1, y2))), bbox_hr(ceil(threeMax(x0, x1, x2)), ceil(threeMax(y0, y1, y2)));
    Vector3D pt_bary_00;
    // bool get=((color.b==0.000000)&&(bbox_ll.x==85.000000)&&(bbox_ll.y==85.000000)&&(bbox_hr.x==939.000000)&&(bbox_hr.y==939.000000));
    // bool bug;
    // if (get) printf("color %f %f %f\n", color.r, color.g, color.b);
    for (double i = bbox_ll.x; i <= bbox_hr.x; i=i+1.0) {
      for (double j = bbox_ll.y; j <= bbox_hr.y; j=j+1.0) {
        // judge if need supsamping
        for ( double supsami = pix_siz/2; supsami < 1; supsami=supsami+pix_siz ) {
          for ( double supsamj = pix_siz/2; supsamj < 1; supsamj=supsamj+pix_siz ) {
            // super-sample within a original sample
            pt[0] = i+supsami; pt[1] = j+supsamj;
            // if (this->sample_rate == 4) printf("inin %f %f\n", i, j);
            baryCompute_cgl(pt, a, b, c, pt_bary_00);
            if (pt_bary_00[0]>=-4.94065645841246544E-8 && pt_bary_00[1]>=-4.94065645841246544E-8 && pt_bary_00[2]>=-4.94065645841246544E-10) {
              // if covered, supsamp
              fill_pixel(round((pt[0]-(pix_siz/2))/pix_siz), round((pt[1]-(pix_siz/2))/pix_siz), color);
              // if (get&&bug) printf("pt %f %f\n", pt[0], pt[1]);
              // if (get&&bug) fill_pixel(round((pt[0]-(pix_siz/2))/pix_siz), round((pt[1]-(pix_siz/2))/pix_siz), Color(1,0,0));
            }
          }
        }

      }
    }
  }


  void RasterizerImp::rasterize_interpolated_color_triangle(float x0, float y0, Color c0,
    float x1, float y1, Color c1,
    float x2, float y2, Color c2)
  {
    double pix_siz= 1.0/sqrt(this->sample_rate);
    Vector2D a(x0,y0), b(x1,y1), c(x2,y2), pt; 
    Vector2D bbox_ll(floor(threeMin(x0, x1, x2)), floor(threeMin(y0, y1, y2))), bbox_hr(ceil(threeMax(x0, x1, x2)), ceil(threeMax(y0, y1, y2)));
    Vector3D pt_bary_00;
    Color interpo_color;
    // printf("pizs %f\n", pix_siz);
    for (double i = bbox_ll.x; i < bbox_hr.x; i=i+1.0) {
      // printf("in %f\n", i);
      for (double j = bbox_ll.y; j < bbox_hr.y; j=j+1.0) {
        // judge if need supsamping
        for ( double supsami = pix_siz/2; supsami < 1; supsami=supsami+pix_siz ) {
          for ( double supsamj = pix_siz/2; supsamj < 1; supsamj=supsamj+pix_siz ) {
            // super-sample within a original sample
            pt[0] = i+supsami; pt[1] = j+supsamj;
            // if (this->sample_rate == 4) printf("inin %f %f\n", i, j);
            // if (this->sample_rate == 9) printf("inin %f %f\n", pt[0], pt[1]);
            baryCompute_cgl(pt, a, b, c, pt_bary_00);
            // printf("rastering pt %f, %f\n", pt[0], pt[1]);
            // printf("rastering bary %f, %f, %f\n", pt_bary[0], pt_bary[1], pt_bary[2]);
            // printf("bary dn\n", j);
            if (pt_bary_00[0]>=0 && pt_bary_00[1]>=0 && pt_bary_00[2]>=0) {
              // if covered, supsamp
              // fill supersampled pixel in sample buffer 
              // if (this->sample_rate == 9) printf("pixfill pt %f, %f\n", (pt[0]-(pix_siz/2))/pix_siz, (pt[1]-(pix_siz/2))/pix_siz);
              interpo_color = pt_bary_00[0]*c0 + pt_bary_00[1]*c1 + pt_bary_00[2]*c2;
              fill_pixel(round((pt[0]-(pix_siz/2))/pix_siz), round((pt[1]-(pix_siz/2))/pix_siz), interpo_color);
            }
          }
        }

      }
    }
  }


  void RasterizerImp::rasterize_textured_triangle(float x0, float y0, float u0, float v0,
    float x1, float y1, float u1, float v1,
    float x2, float y2, float u2, float v2,
    Texture& tex)
  {
    // TODO: Task 5: Fill in the SampleParams struct and pass it to the tex.sample function.
    // printf("smp at %f %f %f %f %f %f\n", x0,y0,x1,y1,x2,y2);
    SampleParams sp;
    sp.psm = this->psm;
    sp.lsm = this->lsm;

    // vector<float> pt(2), pt_bary(3), tri(6), bbox(4);
    // tri[0] = x0;
    // tri[1] = y0;
    // tri[2] = x1;
    // tri[3] = y1;
    // tri[4] = x2;
    // tri[5] = y2;
    // // printf("min max has %f %f %f %f\n", floor(threeMin(x0, x1, x2)), threeMin(y0, y1, y2), ceil(threeMax(x0, x1, x2)), threeMax(y0, y1, y2));
    // bbox[0] = floor(threeMin(x0, x1, x2));
    // bbox[1] = floor(threeMin(y0, y1, y2));
    // bbox[2] = ceil(threeMax(x0, x1, x2));
    // bbox[3] = ceil(threeMax(y0, y1, y2));

    // printf("triang sampling stt bb %d %d %d %d\n", bbox[0], bbox[1], bbox[2], bbox[3]);
    double pix_siz= 1.0/sqrt(this->sample_rate);
    Color res_col;
    Vector2D pt_00, pt_10, pt_01, a(x0,y0), b(x1,y1), c(x2,y2); 
    Vector2D bbox_ll(floor(threeMin(x0, x1, x2)), floor(threeMin(y0, y1, y2))), bbox_hr(ceil(threeMax(x0, x1, x2)), ceil(threeMax(y0, y1, y2)));
    Vector2D a_tex(u0,v0), b_tex(u1,v1), c_tex(u2,v2);
    Vector3D pt_bary_00, pt_bary_10, pt_bary_01;

    // printf("pizs %f\n", pix_siz);
    for (double i = bbox_ll[0]; i < bbox_hr[0]; i=i+1.0) {
      // printf("in %f\n", i);
      for (double j = bbox_ll[1]; j < bbox_hr[1]; j=j+1.0) {
        // judge if need supsamping
        for ( double supsami = pix_siz/2; supsami < 1; supsami=supsami+pix_siz ) {
          for ( double supsamj = pix_siz/2; supsamj < 1; supsamj=supsamj+pix_siz ) {
            // super-sample within a original sample
            pt_00[0]=i+supsami; pt_00[1]=j+supsamj;
            // if (this->sample_rate == 4) printf("inin %f %f\n", i, j);
            // if (this->sample_rate == 9) printf("inin %f %f\n", pt[0], pt[1]);
            baryCompute_cgl(pt_00, a, b, c, pt_bary_00);
            // printf("rastering pt %f, %f\n", pt[0], pt[1]);
            // printf("rastering bary %f, %f, %f\n", pt_bary[0], pt_bary[1], pt_bary[2]);
            // printf("bary dn\n", j);
            if (pt_bary_00.x>=0 && pt_bary_00.y>=0 && pt_bary_00.z>=0) {
              // if covered, supsamp
              // fill supersampled pixel in sample buffer 
              // if (this->sample_rate == 9) printf("pixfill pt %f, %f\n", (pt[0]-(pix_siz/2))/pix_siz, (pt[1]-(pix_siz/2))/pix_siz);
              sp.p_uv = pt_bary_00.x * a_tex + pt_bary_00.y * b_tex + pt_bary_00.z * c_tex;
              
              // compute diffs
              pt_10 = pt_00 + Vector2D(1,0);
              pt_01 = pt_00 + Vector2D(0,1);
              baryCompute_cgl(pt_10, a, b, c, pt_bary_10);
              baryCompute_cgl(pt_01, a, b, c, pt_bary_01);
              sp.p_dy_uv = (pt_bary_01).x * a_tex + (pt_bary_01).y * b_tex + (pt_bary_01).z * c_tex;
              sp.p_dx_uv = (pt_bary_10).x * a_tex + (pt_bary_10).y * b_tex + (pt_bary_10).z * c_tex;
              res_col = tex.sample(sp);

              // printf("col is %f %f %f\n", res_col.r, res_col.g, res_col.b);
              fill_pixel(round((pt_00[0]-(pix_siz/2))/pix_siz), round((pt_00[1]-(pix_siz/2))/pix_siz), res_col);
            }
          }
        }

      }
    }
    // struct SampleParams {
    //   Vector2D          p_uv;
    //   Vector2D p_dx_uv, p_dy_uv;
    //   PixelSampleMethod psm;
    //   LevelSampleMethod lsm;
    // };

    // dr->rasterize_textured_triangle(p0_scr.x, p0_scr.y, this->p0_uv.x, this->p0_uv.y,
    //                                 p1_scr.x, p1_scr.y, this->p1_uv.x, this->p1_uv.y,
    //                                 p2_scr.x, p2_scr.y, this->p2_uv.x, this->p2_uv.y,
    //                                 *this->tex);
    // TODO: Task 6: Set the correct barycentric differentials in the SampleParams struct.
    // Hint: You can reuse code from rasterize_triangle/rasterize_interpolated_color_triangle
  }

  void RasterizerImp::set_sample_rate(unsigned int rate) {
    // TODO: Task 2: You may want to update this function for supersampling support
    // printf("sprate setting!!!\n");

    this->sample_rate = rate;

    this->sample_buffer.resize(width * height * rate, Color::White);
  }


  void RasterizerImp::set_framebuffer_target(unsigned char* rgb_framebuffer,
    size_t width, size_t height)
  {
    // TODO: Task 2: You may want to update this function for supersampling support

    this->width = width;
    this->height = height;
    this->rgb_framebuffer_target = rgb_framebuffer;


    this->sample_buffer.resize(width * height * this->sample_rate, Color::White);
  }


  void RasterizerImp::clear_buffers() {
    std::fill(rgb_framebuffer_target, rgb_framebuffer_target + 3 * width * height, 255);
    std::fill(sample_buffer.begin(), sample_buffer.end(), Color::White);
  }


  // This function is called at the end of rasterizing all elements of the
  // SVG file.  If you use a supersample buffer to rasterize SVG elements
  // for antialising, you could use this call to fill the target framebuffer
  // pixels from the supersample buffer data.
  //
  void RasterizerImp::resolve_to_framebuffer() {
    // TODO: Task 2: You will likely want to update this function for supersampling support
    float eff_wid = width * sqrt(this->sample_rate);

    for (int x = 0; x < width; x=x+1) {
      for (int y = 0; y < height; y=y+1) {
        // box filtering sample buffer
        Color res_col(0,0,0);
        for (int i = 0; i < sqrt(this->sample_rate); i++) {
          for (int j = 0; j < sqrt(this->sample_rate); j++) {
            res_col = res_col + sample_buffer[((i + y * sqrt(this->sample_rate)) * eff_wid) + (j + x * sqrt(this->sample_rate))];
          }          
        }
        res_col = res_col * (1.0/this->sample_rate);
        // color has RGB
        for (int k = 0; k < 3; ++k) {
          // rgb spanned
          this->rgb_framebuffer_target[3 * (y * width + x) + k] = (&res_col.r)[k] * 255;
        }
      }
    }

  }

  Rasterizer::~Rasterizer() { }


}// CGL
