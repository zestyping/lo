#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

#include "libfreenect.h"
#include "opc.h"

pthread_mutex_t frame_ready_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t frame_ready_cond = PTHREAD_COND_INITIALIZER;
int frame_ready = 0;

pixel frame1[640*480], frame2[640*480];

// "g_" variables belong to the GLUT thread; "f_" to the Freenect thread.
pthread_t f_thread;
freenect_context* f_context;
freenect_device* f_device;
volatile int f_quit = 0, g_quit = 0;
pixel* f_frame = frame1;
pixel* g_frame = frame2;
int g_window;
GLuint g_texture;

#define SWAP(type, a, b) { type c = a; a = b; b = c; }

void g_init(int width, int height) {
  // Set GL options.
  glClearColor(0, 0, 0, 0);
  glClearDepth(1);
  glDepthFunc(GL_LESS);
  glDepthMask(GL_FALSE);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
  glDisable(GL_ALPHA_TEST);
  glEnable(GL_TEXTURE_2D);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_FLAT);

  // Set up the projection.
  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, 640, 480, 0, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Prepare a texture for displaying the frame.
  glGenTextures(1, &g_texture);
  glBindTexture(GL_TEXTURE_2D, g_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void g_exit();

void g_display() {
  if (g_quit) {
    g_exit();
  }

  // Wait for the next frame to be ready in f_frame, then swap it into g_frame.
  pthread_mutex_lock(&frame_ready_mutex);
  while (!frame_ready) {
    pthread_cond_wait(&frame_ready_cond, &frame_ready_mutex);
  }
  SWAP(pixel*, f_frame, g_frame);
  frame_ready = 0;
  pthread_mutex_unlock(&frame_ready_mutex);

  // Paint g_frame into the display buffer.
  glBindTexture(GL_TEXTURE_2D, g_texture);
  glTexImage2D(
      GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, g_frame);
  glBegin(GL_TRIANGLE_FAN);
  glColor4f(1, 1, 1, 1);
  glTexCoord2f(0, 0);
  glVertex3f(0, 0, 0);
  glTexCoord2f(1, 0);
  glVertex3f(640, 0, 0);
  glTexCoord2f(1, 1);
  glVertex3f(640, 480, 0);
  glTexCoord2f(0, 1);
  glVertex3f(0, 480, 0);
  glEnd();
  glutSwapBuffers();
}

void g_keypress(unsigned char key, int x, int y) {
  if (key == 27) {
    g_quit = 1;
  }
}

void* g_main(void* arg) {
  int argc = 0;
  char* argv = NULL;

  glutInit(&argc, &argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  glutInitWindowSize(640, 480);
  glutInitWindowPosition(0, 0);
  g_window = glutCreateWindow("depth camera");
  glutDisplayFunc(g_display);
  glutIdleFunc(g_display);
  glutKeyboardFunc(g_keypress);
  g_init(640, 480);
  glutMainLoop();
  return NULL;
}

typedef struct {
  struct timeval time;
  u16 depth[640*480];
} frame;

#define MAX_FRAMES 300
int num_frames = 0;
frame* frames;
FILE* record_fp;

void g_exit() {
  int f;

  f_quit = 1;
  pthread_join(f_thread, NULL);
  glutDestroyWindow(g_window);

  fprintf(stderr, "Writing frames");
  for (f = 0; f < num_frames; f++) {
    fprintf(stderr, ".");
    fwrite(&(frames[f]), sizeof(frame), 1, record_fp);
  }
  fprintf(stderr, " done.\n");
  exit(0);
}

#define set_pixel(p, nr, ng, nb) ((p).r = nr, (p).g = ng, (p).b = nb)

void f_draw(u16* depth) {
  for (int i = 0; i < 640*480; i++) {
    int v = depth[i] * (256*6) / 1090;
    int hi = v >> 8, lo = v & 0xff;
    pixel p;
    hi == 0 ? set_pixel(p, 255, 255 - lo, 255 - lo) :
        hi == 1 ? set_pixel(p, 255, lo, 0) :
        hi == 2 ? set_pixel(p, 255 - lo, 255, 0) :
        hi == 3 ? set_pixel(p, 0, 255, lo) :
        hi == 4 ? set_pixel(p, 0, 255 - lo, 255) :
        hi == 5 ? set_pixel(p, 0, 0, 255 - lo) :
        set_pixel(p, 0, 0, 0);
    f_frame[i] = p;
  }
}

void f_depth_callback(freenect_device* dev, void* data, u32 timestamp) {
  pthread_mutex_lock(&frame_ready_mutex);
  if (num_frames < MAX_FRAMES) {
    gettimeofday(&(frames[num_frames].time), NULL);
    memcpy(frames[num_frames].depth, data, 640*480*sizeof(u16));
    num_frames++;
    fprintf(stderr, "%d\n", num_frames);
  } else {
    g_quit = 1;
  }
  f_draw((u16*) data);
  frame_ready = 1;
  pthread_cond_signal(&frame_ready_cond);
  pthread_mutex_unlock(&frame_ready_mutex);
}

void* f_main(void* arg) {
  freenect_set_led(f_device, LED_OFF);
  freenect_set_depth_callback(f_device, f_depth_callback);
  freenect_set_depth_mode(f_device, freenect_find_depth_mode(
      FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
  freenect_start_depth(f_device);
  while (!f_quit && freenect_process_events(f_context) >= 0);
  freenect_stop_depth(f_device);
  freenect_close_device(f_device);
  freenect_shutdown(f_context);
  return NULL;
}

int main(int argc, char** argv) {
  record_fp = fopen(argv[1], "w");
  if (!record_fp) {
    fprintf(stderr, "Usage: %s <filename>\n", argv[0]);
    exit(1);
  }

  // Open Freenect device 0.
  if (freenect_init(&f_context, NULL) < 0) {
    fprintf(stderr, "freenect_init failed\n");
    return 1;
  }
  freenect_set_log_level(f_context, FREENECT_LOG_DEBUG);
  freenect_select_subdevices(
      f_context, FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA);
  if (freenect_open_device(f_context, &f_device, 0) < 0) {
    fprintf(stderr, "freenect_open_device failed\n");
    return 1;
  }

  frames = malloc(MAX_FRAMES*sizeof(frame));

  // Start both threads.
  pthread_create(&f_thread, NULL, f_main, NULL);
  g_main(NULL); // Mac OS X requires GLUT to run on the main thread
  return 0;
}
