/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "libfreenect.h"

#include <pthread.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <math.h>

#include "opc.h"

opc_sink sink;
int rows, cols;
pixel* pixels;
//#define pixel_rc(r, c) pixels[(c)*(rows) + (((c) % 2) ? (r) : (rows - 1 - (r)))]
#define pixel_rc(r, c) pixels[(c)*(rows) + (((c + 1) % 2) ? (r) : (rows - 1 - (r)))]
int* col_depths;
int *col_ys;
int *last_col_ys;

#define MAX_PARTICLES 2000

typedef struct {
  float c, r;
  float vr;
  float hue, sat, val;
} particle;

int num_particles = 0;
particle particles[MAX_PARTICLES];
float depth_max = 3;
float min_emit_vr = 5;
float vr_friction = 0.05;
float emit_velocity = 0.05;
float emit_hue = 2;
float emit_value = 0.03;
float val_decay = 0.95;

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

// back: owned by libfreenect (implicit for depth)
// mid: owned by callbacks, "latest frame ready"
// front: owned by GL, "currently being drawn"
uint8_t *depth_mid, *depth_front;

GLuint gl_depth_tex;

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_angle = 0;
int freenect_led;

pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_depth = 0;

void DrawGLScene() {
  pthread_mutex_lock(&gl_backbuf_mutex);

  while (!got_depth) {
    pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
  }

  uint8_t *tmp;

  if (got_depth) {
    tmp = depth_front;
    depth_front = depth_mid;
    depth_mid = tmp;
    got_depth = 0;
  }

  pthread_mutex_unlock(&gl_backbuf_mutex);

  glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_front);

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
  glTexCoord2f(0, 0); glVertex3f(0,0,0);
  glTexCoord2f(1, 0); glVertex3f(640,0,0);
  glTexCoord2f(1, 1); glVertex3f(640,480,0);
  glTexCoord2f(0, 1); glVertex3f(0,480,0);
  glEnd();

  glutSwapBuffers();
}

void keyPressed(unsigned char key, int x, int y) {
  if (key == 27) {
    die = 1;
    pthread_join(freenect_thread, NULL);
    glutDestroyWindow(window);
    free(depth_mid);
    free(depth_front);
    exit(0);
  }
  if (key == 'w') {
    freenect_angle++;
    if (freenect_angle > 30) {
      freenect_angle = 30;
    }
  }
  if (key == 's') {
    freenect_angle = 0;
  }
  if (key == 'x') {
    freenect_angle--;
    if (freenect_angle < -30) {
      freenect_angle = -30;
    }
  }
  if (key == '1') {
    freenect_set_led(f_dev,LED_GREEN);
  }
  if (key == '2') {
    freenect_set_led(f_dev,LED_RED);
  }
  if (key == '3') {
    freenect_set_led(f_dev,LED_YELLOW);
  }
  if (key == '4') {
    freenect_set_led(f_dev,LED_BLINK_GREEN);
  }
  if (key == '5') {
    // 5 is the same as 4
    freenect_set_led(f_dev,LED_BLINK_GREEN);
  }
  if (key == '6') {
    freenect_set_led(f_dev,LED_BLINK_RED_YELLOW);
  }
  if (key == '0') {
    freenect_set_led(f_dev,LED_OFF);
  }
  freenect_set_tilt_degs(f_dev,freenect_angle);

  if (strchr("[]azsxdcfvgbhn", key)) {
    switch (key) {
      case '[': depth_max -= 0.1; break;
      case ']': depth_max += 0.1; break;
      case 'a': min_emit_vr += 0.1; break;
      case 'z': min_emit_vr -= 0.1; break;
      case 's': vr_friction += 0.01; break;
      case 'x': vr_friction -= 0.01; break;
      case 'd': emit_velocity += 0.01; break;
      case 'c': emit_velocity -= 0.01; break;
      case 'f': emit_hue += 0.01; break;
      case 'v': emit_hue -= 0.01; break;
      case 'g': emit_value += 0.01; break;
      case 'b': emit_value -= 0.01; break;
      case 'h': val_decay += 0.01; break;
      case 'n': val_decay -= 0.01; break;
    }
    fprintf(stderr,
    "depth_max:%4.1f  min_emit_vr:%4.2f  vr_friction:%4.2f\n"
    " emit_vel:%4.2f  emit_hue:   %4.2f  emit_value: %4.2f  val_decay:%4.2f\n",
    depth_max, min_emit_vr, vr_friction,
    emit_velocity, emit_hue, emit_value, val_decay);
  }
}

void ReSizeGLScene(int Width, int Height) {
  glViewport(0,0,Width,Height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 640, 480, 0, -1.0f, 1.0f);
  glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void InitGL(int Width, int Height) {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glEnable(GL_TEXTURE_2D);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_FLAT);

  glGenTextures(1, &gl_depth_tex);
  glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  ReSizeGLScene(Width, Height);
}

void *gl_threadfunc(void *arg) {
  printf("GL thread\n");

  glutInit(&g_argc, g_argv);

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  glutInitWindowSize(640, 480);
  glutInitWindowPosition(0, 0);

  window = glutCreateWindow("LibFreenect");

  glutDisplayFunc(&DrawGLScene);
  glutIdleFunc(&DrawGLScene);
  glutReshapeFunc(&ReSizeGLScene);
  glutKeyboardFunc(&keyPressed);

  InitGL(640, 480);

  glutMainLoop();

  return NULL;
}

uint16_t t_gamma[2048];

void set_rgb_by_depth(uint16_t depth, uint8_t* rgb) {
  int pval = t_gamma[depth];
  int lb = pval & 0xff;
  switch (pval>>8) {
    case 0:
      rgb[0] = 255;
      rgb[1] = 255-lb;
      rgb[2] = 255-lb;
      break;
    case 1:
      rgb[0] = 255;
      rgb[1] = lb;
      rgb[2] = 0;
      break;
    case 2:
      rgb[0] = 255-lb;
      rgb[1] = 255;
      rgb[2] = 0;
      break;
    case 3:
      rgb[0] = 0;
      rgb[1] = 255;
      rgb[2] = lb;
      break;
    case 4:
      rgb[0] = 0;
      rgb[1] = 255-lb;
      rgb[2] = 255;
      break;
    case 5:
      rgb[0] = 0;
      rgb[1] = 0;
      rgb[2] = 255-lb;
      break;
    default:
      rgb[0] = 0;
      rgb[1] = 0;
      rgb[2] = 0;
      break;
  }
}

pixel hue_pixel(float hue) {
  pixel p;
  int k = (hue - floor(hue)) * 255 * 3;
  if (k < 255) {
    p.r = 255 - k;
    p.g = k;
    p.b = 0;
  } else if (k < 510) {
    p.g = 255 - (k - 255);
    p.b = k - 255;
    p.r = 0;
  } else {
    p.b = 255 - (k - 510);
    p.r = k - 510;
    p.g = 0;
  }
  return p;
}

void advance_particles() {
  int i;
  particle* p;
  for (i = 0, p = particles; i < num_particles; i++, p++) {
    p->r += p->vr;
    p->val *= 0.95;
    if (p->vr > vr_friction) p->vr -= vr_friction;
    else if (p->vr < -vr_friction) p->vr += vr_friction;
    else p->vr = 0;
    if (p->val < 0.001 || p->r < -50 || p->r > rows + 50) {
      *p = particles[--num_particles];
      i--;
      p--;
    }
  }
}

u8 clamp_byte(float val) {
  return (val < 0) ? 0 : (val > 255) ? 255 : val;
}

void draw_particles() {
  int i, r, c;
  float d, v;
  particle* p;
  pixel* px;
  pixel dpx;
  bzero(pixels, rows*cols*sizeof(pixel));
  for (i = 0, p = particles; i < num_particles; i++, p++) {
    for (r = 0; r < rows; r++) {
      c = p->c;
      d = p->r - r;
      v = p->val/(1 + d*d);
      dpx = hue_pixel(p->hue);
      px = &pixel_rc(r, c);
      px->r = clamp_byte((float) px->r + v*dpx.r);
      px->g = clamp_byte((float) px->g + v*dpx.g);
      px->b = clamp_byte((float) px->b + v*dpx.b);
    }
  }
}

#define depth_m(d) ((d) > 1089 ? 100 : (0.1236*tan((d)/2842.5 + 1.1863)))

void emit_particles() {
  particle* p;
  int r, c, vr;

  for (c = 0; c < cols; c++) {
    if (col_ys[c] != 0 && last_col_ys[c] != 0) {
      vr = col_ys[c] - last_col_ys[c];
      if (fabs(vr) > min_emit_vr) {
        p = &(particles[num_particles++]);
        p->c = c;
        p->r = (0.5 + (col_ys[c]/480.0) * 0.5) * rows;
        p->r = rows;
        p->vr = vr*emit_velocity;
        p->hue = depth_m(col_depths[c])*emit_hue;
        p->sat = 1;
        p->val = vr*emit_value;
        fprintf(stderr, "emit: @%.1f,%.1f v=%3.1f hue=%4.2f val=%4.1f %5d  \n",
                p->c, p->r, p->vr, p->hue, p->val, col_depths[c]);
      }
    }
  }
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp) {
  int i, x, y, r, c, last_c;
  uint16_t *depth = (uint16_t*)v_depth;
  int top_ys[640], top_depths[640];
  float depth_last, depth_here;

  pthread_mutex_lock(&gl_backbuf_mutex);
  bzero(depth_mid, 640*480*3*sizeof(uint8_t));
  for (x = 0; x < 640; x++) {
    depth_last = depth_m(depth[x]);
    top_ys[x] = 0;
    top_depths[x] = 0;
    for (y = 1; y < 480; y++) {
      i = x + y*640;
      depth_here = depth_m(depth[i]);
      if (depth_here < depth_max && depth_here < depth_last - 0.3) {
        top_ys[x] = y;
        top_depths[x] = depth[i];
        depth_mid[3*i + 1] = 255;
      }
      depth_last = depth_here;
    }
  }

  for (c = 0; c < cols; c++) {
    col_depths[c] = 0;
    col_ys[c] = 0;
    i = 0;
    for (x = c*640/cols; x < (c + 1)*640/cols; x++) {
      if (top_ys[x] > 0 && top_depths[x] > 0) {
        col_depths[c] += top_depths[x];
        col_ys[c] += top_ys[x];
        i++;
      }
    }
    if (i > 0) {
      col_depths[c] /= i;
      col_ys[c] /= i;
    }
  }

  got_depth++;
  pthread_cond_signal(&gl_frame_cond);
  pthread_mutex_unlock(&gl_backbuf_mutex);

  emit_particles();
  advance_particles();
  draw_particles();
  opc_put_pixels(sink, 1, rows*cols, pixels);

  fprintf(stderr, "particles: %d      \r", num_particles);
  fflush(stderr);
  memcpy(last_col_ys, col_ys, sizeof(int)*cols);
}

void *freenect_threadfunc(void *arg) {
  freenect_set_tilt_degs(f_dev,freenect_angle);
  freenect_set_led(f_dev,LED_RED);
  freenect_set_depth_callback(f_dev, depth_cb);
  freenect_set_depth_mode(f_dev, freenect_find_depth_mode(
      FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));

  freenect_start_depth(f_dev);
  freenect_start_video(f_dev);

  while (!die && freenect_process_events(f_ctx) >= 0) {
  }

  freenect_stop_depth(f_dev);
  freenect_close_device(f_dev);
  freenect_shutdown(f_ctx);

  return NULL;
}

int main(int argc, char **argv) {
  int res;

  sink = opc_new_sink(argv[1], 9876);
  cols = atoi(argv[2]);
  rows = atoi(argv[3]);
  pixels = malloc(sizeof(pixel)*rows*cols);
  bzero(pixels, sizeof(pixel)*rows*cols);

  col_ys = malloc(sizeof(int)*cols);
  last_col_ys = malloc(sizeof(int)*cols);
  bzero(col_ys, sizeof(int)*cols);
  bzero(last_col_ys, sizeof(int)*cols);
  col_depths = malloc(sizeof(int)*cols);

  depth_mid = (uint8_t*)malloc(640*480*3);
  depth_front = (uint8_t*)malloc(640*480*3);

  int i;
  for (i=0; i<2048; i++) {
    float v = i/2048.0;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }

  g_argc = argc;
  g_argv = argv;

  if (freenect_init(&f_ctx, NULL) < 0) {
    printf("freenect_init() failed\n");
    return 1;
  }

  freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
  freenect_select_subdevices(
      f_ctx, FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA);

  int nr_devices = freenect_num_devices (f_ctx);
  printf ("Number of devices found: %d\n", nr_devices);

  int user_device_number = 0;

  if (nr_devices < 1) {
    freenect_shutdown(f_ctx);
    return 1;
  }

  if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
    printf("Could not open device\n");
    freenect_shutdown(f_ctx);
    return 1;
  }

  res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
  if (res) {
    printf("pthread_create failed\n");
    freenect_shutdown(f_ctx);
    return 1;
  }

  // OS X requires GLUT to run on the main thread
  gl_threadfunc(NULL);

  return 0;
}
