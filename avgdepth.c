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
#define pixel_rc(r, c) pixels[(c)*(rows) + (((c + 1) % 2) ? (r) : (rows - 1 - (r)))]
#define old_pixel_rc(r, c) old_pixels[(c)*(rows) + (((c + 1) % 2) ? (r) : (rows - 1 - (r)))]

opc_sink sink;
int rows, cols;
pixel* pixels;

pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int max_depth = 1090;

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
  if (key == '[') {
    max_depth--;
  }
  if (key == ']') {
    max_depth++;
  }
  freenect_set_tilt_degs(f_dev,freenect_angle);
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
  if (depth == 0) {
    rgb[0] = 0;
    rgb[1] = 0;
    rgb[2] = 0;
    return;
  }
  int pval = t_gamma[depth];
  pval = (pval*8) % (256*6);
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
  rgb[0] >>= 2;
  rgb[1] >>= 2;
  rgb[2] >>= 2;
}

pixel old_pixels[625];

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp) {
  int i, r, c, x, y;
  float avg;
  uint16_t *depth = (uint16_t*)v_depth;
  uint16_t d;
  pixel px;
  pixel* np;
  pixel* op;
  float f = 0.25;

  pthread_mutex_lock(&gl_backbuf_mutex);
  for (i = 0; i < 640*480; i++) {
    if (depth[i] < max_depth) {
      set_rgb_by_depth(depth[i], &(depth_mid[3*i]));
    } else {
      set_rgb_by_depth(0, &(depth_mid[3*i]));
    }
  }
  for (r = 0; r < rows; r++) {
    for (c = 0; c < cols; c++) {
      //set_rgb_by_depth(depth[(640*c/cols) + 640*(480*r/rows)],
      //                 (uint8_t*) &(pixels[r*cols + c]));
      i = 0;
      avg = 0;
      for (x = (640*c/cols); x < (640*(c+1)/cols); x++) {
        for (y = 240+(240*r/rows); y < 240+(240*(r+1)/rows); y++) {
          d = depth[x + 640*y];
          if (d < max_depth) {
            i++;
            avg += d;
          }
        }
      }
      if (i > 0) {
        avg = avg / i;
      }
      set_rgb_by_depth(avg, &px);
      //op = (uint8_t*) &(old_pixels[r*cols + c]);
      //np = (uint8_t*) &(pixels[r*cols + c]);
      op = &(old_pixel_rc(r, c));
      np = &(pixel_rc(r, c));
      np->r = op->r * (1 - f) + px.r*f;
      np->g = op->g * (1 - f) + px.g*f;
      np->b = op->b * (1 - f) + px.b*f;
    }
  }

  got_depth++;
  pthread_cond_signal(&gl_frame_cond);
  pthread_mutex_unlock(&gl_backbuf_mutex);

  opc_put_pixels(sink, 1, rows*cols, pixels);

  memcpy(old_pixels, pixels, 625*sizeof(pixel));
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
