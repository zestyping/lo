#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "libfreenect.h"
#include "opc.h"

#define SWAP(type, a, b) { type c = a; a = b; b = c; }
#define depth_to_mm(d) (1000/(-0.00307*d + 3.33))

// These variables are shared between both threads.
pthread_mutex_t depth_ready_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t depth_ready_cond = PTHREAD_COND_INITIALIZER;
int depth_ready = 0;
u16 depth1[640*480], depth2[640*480];

// "f_" variables belong to the Freenect thread.
pthread_t f_thread;
volatile int f_should_quit = 0;
freenect_context* f_context;
freenect_device* f_device;
u16* f_depth = depth1;
volatile int f_paused = 0;

// "g_" variables belong to the GLUT thread
volatile int g_should_quit = 0;
u16* g_depth = depth2;
int g_window;
GLuint g_texture;
opc_sink g_sink;

typedef struct {
  u16 altitude, depth_mm;
  double depth_m;
} col_record;

typedef struct {
  char* name;
  char* format;
  float value;
  float delta;
  float min, max;
  int modulo;
} param;

param g_params[100] = {
  { "min_depth", "%4.2f m", 0.5, 0.1, 0.1, 9.0, 0 },
  { "max_depth", "%4.2f m", 2.7, 0.1, 0.1, 9.0, 0 },
  { "x_width", "%3.0f", 640, 1, 10, 640, 0 },
  { "y_height", "%3.0f", 460, 1, 10, 480, 0 },

  { "c_flip", "%.0f", 1, 1, 0, 1, 2 },
  { "r_shift", "%+3.0f", 0, 1, -50, 50, 0 },
  { "cam_rot", "%.0f", 0, 1, 0, 3, 4 },
  { "y_shift", "%+3.0f", 0, 1, -80, 80, 0},

  { "emit_min_v", "%4.1f", 10.0, 0.1, 0.1, 100, 0},
  { "emit_velf", "%4.2f", 0.05, 0.01, 0, 1, 0},
  { "friction", "%4.2f", 0.05, 0.01, 0, 1, 0},
  { "depth_step", "%4.2f m", 0.15, 0.10, 0, 10, 0},

  { "hue_cycles", "%3.1f", 1, 0.1, 0.1, 4, 0},
  { "emit_valf", "%5.3f", 0.030, 0.01, 0, 1, 0},
  { "val_decay", "%5.3f", 0.950, 0.01, 0, 1, 0},
  { "max_val", "%3.0f", 255, 10, 0, 255, 0 },

  { NULL, NULL, 0, 0 }
};
#define min_depth g_params[0].value
#define max_depth g_params[1].value
#define x_width g_params[2].value
#define y_height g_params[3].value

#define c_flip g_params[4].value
#define r_shift g_params[5].value
#define cam_rot g_params[6].value
#define y_shift g_params[7].value

#define emit_min_v g_params[8].value
#define emit_velf g_params[9].value
#define friction g_params[10].value
#define depth_step g_params[11].value

#define hue_cycles g_params[12].value
#define emit_valf g_params[13].value
#define val_decay g_params[14].value
#define max_val g_params[15].value

int g_num_params = 0;
int g_selected_param = 0;
int rows = 50, cols = 25;

int pixel_map[50][25];

// Pixel adjustments.
int g_num_pixel_ranges = 0;
struct {
  int start, stop;
} g_pixel_ranges[100];

// Particles.
#define MAX_PARTICLES 2000

typedef struct {
  float c, r, v;
  float hue, sat, val;
} particle;

int num_particles = 0;
particle particles[MAX_PARTICLES];

// Pure functions.
u8 clamp_byte(float val) {
  return (val < 0) ? 0 : (val > 255) ? 255 : val;
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

// GLUT thread functions.
void g_show_params() {
  int p;
  char buffer[10];
  fprintf(stderr, "\n\n");
  for (p = 0; p < g_num_params; p++) {
    fprintf(stderr, "%s", p == g_selected_param ? "\x1b[33;1m>" : " ");
    fprintf(stderr, "%10s:", g_params[p].name);
    sprintf(buffer, g_params[p].format, g_params[p].value);
    fprintf(stderr, "%-7s", buffer);
    fprintf(stderr, "%s ", p == g_selected_param ? "\x1b[0m" : "");
  }
  fprintf(stderr, "\n\n");
}

int g_save_params(char* name) {
  FILE* fp = fopen(name, "w");
  int p;
  if (!fp) {
    return 0;
  }
  for (p = 0; p < g_num_params; p++) {
    if (g_params[p].name && g_params[p].name[0]) {
      fprintf(fp, "%s %.6f\n", g_params[p].name, g_params[p].value);
    }
  }
  fclose(fp);
  return 1;
}

int g_load_params(char* name) {
  char param_name[100];
  float param_value;
  FILE* fp = fopen(name, "r");
  int p;
  if (!fp) {
    return 0;
  }
  while (fscanf(fp, "%s %f\n", param_name, &param_value) == 2) {
    for (p = 0; p < g_num_params; p++) {
      if (strcmp(param_name, g_params[p].name) == 0) {
        g_params[p].value = param_value;
        break;
      }
    }
  }
  fclose(fp);
  return 1;
}

void g_init(int width, int height) {
  for (g_num_params = 0; g_params[g_num_params].name; g_num_params++);
  g_load_params("current.params");

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

void g_quit() {
  f_should_quit = 1;
  pthread_join(f_thread, NULL);
  glutDestroyWindow(g_window);
  exit(0);
}

void g_draw_frame(pixel* frame, u16* depth, col_record* col_records) {
  s32 i, d, v;
  u8 hi, lo;
  pixel p, black, white;
  s32 min_mm = min_depth*1000;
  s32 max_mm = max_depth*1000;
  int min_x = (640 - x_width) / 2;
  int max_x = min_x + x_width;
  int min_y = (480 - y_height) / 2;
  int max_y = min_y + y_height;
  int c, x, y;
  pixel frame_oob;

#define set_pixel(p, nr, ng, nb) ((p).r = nr, (p).g = ng, (p).b = nb)

  for (i = 0; i < 640*480; i++) {
    d = depth[i] < min_mm ? min_mm : depth[i] > max_mm ? max_mm : depth[i];
    v = (d - min_mm) * (256*4) / (max_mm - min_mm);
    hi = v >> 8, lo = v & 0xff;
    v == 0 ? set_pixel(p, 255, 255, 255) :
        v == 256*4 ? set_pixel(p, 0, 0, 0) :
        hi == 0 ? set_pixel(p, 255, lo, 0) :
        hi == 1 ? set_pixel(p, 255 - lo, 255, 0) :
        hi == 2 ? set_pixel(p, 0, 255, lo) :
        hi == 3 ? set_pixel(p, 0, 255 - lo, 255) :
        set_pixel(p, 128, 128, 128);
    if (i/640 < min_y || i/640 >= max_y ||
        (i%640) < min_x || (i%640) >= max_x) {
      p.r /= 4;
      p.g /= 4;
      p.b /= 4;
    }
    frame[i] = p;
  }

#define frame_xy(x, y) (*(((x) >= 0 && (x) < 640 && (y) >= 0 && (y) < 480) ? &frame[(x) + (y)*640] : &frame_oob))

  set_pixel(black, 0, 0, 0);
  set_pixel(white, 255, 255, 255);
  for (c = 0; c < 25; c++) {
    for (x = min_x + (x_width*c/25); x < min_x + (x_width*(c + 1)/25); x++) {
      y = 479 - col_records[c].altitude;
      frame_xy(x, y - 1) = black;
      frame_xy(x, y) = white;
      frame_xy(x, y + 1) = black;
    }
  }
}

void g_advance_particles() {
  int i;
  particle* p;
  for (i = 0, p = particles; i < num_particles; i++, p++) {
    p->r += p->v;
    p->val *= val_decay;
    p->v += p->v > friction ? -friction : p->v < -friction ? friction : -p->v;
    if (p->val < 0.001 || p->r < -50 || p->r > rows + 50) {
      *p = particles[--num_particles];
      i--;
      p--;
    }
  }
}

static pixel dummy;
#define pixel_rc(r, c) (pixels[pixel_map[r][c_flip ? 24 - c : c]])

void g_put_pixel_ranges(pixel* pixels) {
  pixel blacks[1250];
  pixel outs[5000];
  bzero(blacks, rows*cols*sizeof(pixel));

  if (g_num_pixel_ranges) {
    int out = 0, start, stop, i;
    for (i = 0; i < g_num_pixel_ranges; i++) {
      start = g_pixel_ranges[i].start;
      stop = g_pixel_ranges[i].stop;
      if (start < stop) {
        memcpy(outs + out, pixels + start, (stop - start)*sizeof(pixel));
        out += stop - start;
      } else {
        memcpy(outs + out, blacks, (start - stop)*sizeof(pixel));
        out += start - stop;
      }
    }
    opc_put_pixels(g_sink, 1, out, outs);
  } else {
    opc_put_pixels(g_sink, 1, 1250, pixels);
  }
}

void g_draw_particles() {
  int i, r, c, shifted_r;
  float d, v;
  particle* p;
  pixel* px;
  pixel dpx;
  pixel pixels[1250];

  bzero(pixels, rows*cols*sizeof(pixel));
  for (i = 0, p = particles; i < num_particles; i++, p++) {
    for (r = 0; r < rows; r++) {
      c = p->c;
      d = p->r - r;
      v = p->val/(1 + d*d);
      dpx = hue_pixel(p->hue);

      shifted_r = r + r_shift;
      if (shifted_r >= 0 && shifted_r < 50) {
        px = &pixel_rc(shifted_r, c);
        px->r = clamp_byte(((float) px->r + v*dpx.r)*max_val/255.99);
        px->g = clamp_byte(((float) px->g + v*dpx.g)*max_val/255.99);
        px->b = clamp_byte(((float) px->b + v*dpx.b)*max_val/255.99);
      }
    }
  }
  g_put_pixel_ranges(pixels);
}

void g_draw_invitation() {
  static float t = 0;
  pixel pixels[1250];
  char play_image[30][25] = {
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "        x              x ",
    "        x              x ",
    "  xxx   x   xx   x  x  x ",
    "  x  x  x     x  x  x  x ",
    "  x  x  x   xxx  x  x  x ",
    "  x  x  x  x  x  x  x    ",
    "  xxx   x   xxx   xxx  x ",
    "  x                 x    ",
    "  x                 x    ",
    "  x              xxx     ",
    "                         ",
    "                         ",
    "                         ",
    "            x            ",
    "            x            ",
    "            x            ",
    "          x x x          ",
    "           xxx           ",
    "            x            ",
    "                         ",
    "                         ",
    "                         "
  };
  char conduct_image[30][25] = {
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "             x         x ",
    "             x         x ",
    " x  x  xx   xx x x  x xxx",
    "x  x x x x x x x x x   x ",
    "x  x x x x x x x x x   x ",
    "x  x x x x x x x x x   x ",
    " x  x  x x  xx  xx  x  x ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "                         ",
    "            x            ",
    "            x            ",
    "            x            ",
    "          x x x          ",
    "           xxx           ",
    "            x            ",
    "                         ",
    "                         ",
    "                         "
  };
  
  int i, j, shifted_r;
  for (i = 0; i < 1250; i++) {
    pixels[i].r = 0;
    pixels[i].g = 0;
    pixels[i].b = 0;
  }

  float rr, gg, bb;
  t += 0.01;
  for (i = 0; i < 30; i++) {
    for (j = 0; j < 25; j++) {
      rr = sin(t*5 + i*0.1 + j*0.02);
      gg = sin(t*3 + i*0.04 - j*0.1 + 1.3);
      bb = sin(t*7 - i*0.07 + j*0.05 + 2.7);
      shifted_r = i + r_shift;
      if (shifted_r >= 0 && shifted_r < 50) {
        if (conduct_image[i][24-j] > 32) {
          pixel_rc(shifted_r, j).r = (int) (max_val*rr);
          pixel_rc(shifted_r, j).g = (int) (max_val*gg);
          pixel_rc(shifted_r, j).b = (int) (max_val*bb);
        }
      }
    }
  }

  g_put_pixel_ranges(pixels);
}

void g_emit_particles(col_record* col_records, col_record* last_col_records) {
  particle* p;
  int r, c;
  float v;
  float depth;
  float depth_frac;

  for (c = 0; c < cols; c++) {
    if (col_records[c].altitude && last_col_records[c].altitude) {
      depth = col_records[c].depth_m;
      v = (col_records[c].altitude - last_col_records[c].altitude)/depth;
      if (fabs(v) > emit_min_v && num_particles < MAX_PARTICLES) {
        p = &(particles[num_particles++]);
        p->c = c;
        p->r = 40 - col_records[c].altitude*20/480;
        p->v = -v*emit_velf;
        p->hue = (depth - min_depth)/(max_depth - min_depth)*hue_cycles;
        p->sat = 1;
        p->val = fabs(v)*emit_valf;
        //if (p->c == 0 || p->c == 24) {
        //  fprintf(stderr, "emit: @%.1f,%.1f v=%3.1f hue=%4.2f val=%4.1f \n",
        //          p->c, p->r, p->v, p->hue, p->val);
        //}
      }
    }
  }
}

void g_draw_pixels(u16* depth) {
  pixel pixels[50*25], p;
  s32 i, d, v;
  u8 hi, lo;
  s32 min_mm = min_depth*1000;
  s32 max_mm = max_depth*1000;
  int r, c, shifted_r;

  bzero(pixels, 25*50*sizeof(pixel));
  for (c = 0; c < 25; c++) {
    for (r = 0; r < 50; r++) {
      d = depth[(r*480/50)*640 + (c*640/25)];
      d = d < min_mm ? min_mm : d > max_mm ? max_mm : d;
      v = (d - min_mm) * (256*4) / (max_mm - min_mm);
      hi = v >> 8, lo = v & 0xff;
      v == 0 ? set_pixel(p, 255, 255, 255) :
          v == 256*4 ? set_pixel(p, 0, 0, 0) :
          hi == 0 ? set_pixel(p, 255, lo, 0) :
          hi == 1 ? set_pixel(p, 255 - lo, 255, 0) :
          hi == 2 ? set_pixel(p, 0, 255, lo) :
          hi == 3 ? set_pixel(p, 0, 255 - lo, 255) :
          set_pixel(p, 128, 128, 128);
      shifted_r = r + r_shift;
      if (shifted_r >= 0 && shifted_r < 50) {
        pixel_rc(shifted_r, c) = p;
      }
    }
  }
  opc_put_pixels(g_sink, 1, 1250, pixels);
}

int compare_samples(const void* a, const void* b) {
  u16 av = ((col_record*) a)->altitude;
  u16 bv = ((col_record*) b)->altitude;
  return av > bv ? 1 : av < bv ? -1 : 0;
}

void g_analyze_columns(u16* depth, col_record* col_records) {
  int x, y, c, i;
  int min_x = (640 - x_width) / 2;
  int min_y = (480 - y_height) / 2;
  int max_y = min_y + y_height;
  col_record samples[50], candidate;
  int num_samples, discard;
  s32 min_mm = min_depth*1000;
  s32 max_mm = max_depth*1000;
  s32 altitude_sum, count;
  double d, last_d;
  double depth_sum;

#define depth_xy(x, y) depth[(x) + (y)*640]/1000.0

  discard = (x_width/25)*0.1;
  discard = (discard < 1) ? 1 : discard;
  for (c = 0; c < 25; c++) {
    num_samples = 0;
    for (x = min_x + (x_width*c/25); x < min_x + (x_width*(c + 1)/25); x++) {
      for (y = min_y; y < max_y; y++) {
        d = depth_xy(x, y);
        if (d <= min_depth || d >= max_depth) break;
      }
      last_d = max_depth;
      candidate.depth_m = 1e9;
      for (; y < max_y; y++) {
        d = depth_xy(x, y);
        if (d > min_depth && d < max_depth) {
          if (last_d - d > depth_step) {  // look for a depth jump
            if (d < candidate.depth_m) {  // pick nearest
              candidate.altitude = 479 - y;
              candidate.depth_m = d;
            }
          }
        }
        last_d = d;
      }
      if (candidate.depth_m < 1e9) {
        samples[num_samples++] = candidate;
      }
    }
    if (num_samples > discard*2 + 2) {
      qsort(samples, num_samples, sizeof(col_record), compare_samples);
      altitude_sum = depth_sum = count = 0;
      for (i = discard; i < num_samples - discard; i++) {
        altitude_sum += samples[i].altitude;
        depth_sum += samples[i].depth_m;
        count++;
      }
      col_records[c].altitude = altitude_sum/count;
      col_records[c].depth_m = depth_sum/count;
      col_records[c].depth_mm = (depth_sum/count) * 1000;
    } else {
      col_records[c].altitude = 0;
      col_records[c].depth_m = 0;
      col_records[c].depth_mm = 0;
    }
  }
}

// Doesn't work if declared local within g_display().  First 10 entries of last_col_records get overwritten with garbage.
static col_record col_records[25], last_col_records[25];

void g_display() {
  static int quiet_frames = 0;
  pixel frame[640*480];

  if (g_should_quit) {
    g_quit();
  }

  // Wait for depth data to be ready in f_depth, then swap it into g_depth.
  if (!depth_ready) return;
  pthread_mutex_lock(&depth_ready_mutex);
  while (!depth_ready) {
    pthread_cond_wait(&depth_ready_cond, &depth_ready_mutex);
  }
  SWAP(u16*, f_depth, g_depth);
  depth_ready = 0;
  pthread_mutex_unlock(&depth_ready_mutex);

  // Extract geometry from the depth frame.
  g_analyze_columns(g_depth, col_records);

  // Emit particles.
  g_emit_particles(col_records, last_col_records);

  // Draw particles from the depth data.
  if (num_particles < 5) {
    quiet_frames++;
  } else {
    quiet_frames = 0;
  }
  if (quiet_frames > 30*10) {
    g_draw_invitation();
  } else {
    g_draw_particles();
  }

  // Advance particles.
  g_advance_particles();
  memcpy(last_col_records, col_records, sizeof(col_record)*25);

  // Draw the frame from the depth data.
  g_draw_frame(frame, g_depth, col_records);

  // Paint the frame into the display buffer.
  glBindTexture(GL_TEXTURE_2D, g_texture);
  glTexImage2D(
      GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, frame);
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

void g_special(int key, int x, int y);

void g_keypress(unsigned char key, int x, int y) {
  char filename[10] = "0.params";
  char* unshifted = "0123456789";
  char* shifted = ")!@#$%^&*(";
  char* p;
  int i;

  if (key == 27) {
    g_should_quit = 1;
  }
  if (key == ' ') {
    f_paused = !f_paused;
  }
  if (p = strchr(unshifted, key)) {
    i = p - unshifted;
    filename[0] = unshifted[i];
    if (g_load_params(filename)) {
      fprintf(stderr, "\nLoaded %s.", filename);
      g_show_params();
    }
    g_save_params("current.params");
  }
  if (p = strchr(shifted, key)) {
    i = p - shifted;
    filename[0] = unshifted[i];
    if (g_save_params(filename)) {
      fprintf(stderr, "\nSaved %s.\n", filename);
    }
  }

  if (key == '=' || key == '+') g_special(GLUT_KEY_UP, x, y);
  if (key == '-' || key == '_') g_special(GLUT_KEY_DOWN, x, y);
  if (key == '[') g_special(GLUT_KEY_LEFT, x, y);
  if (key == ']') g_special(GLUT_KEY_RIGHT, x, y);
}

void g_special(int key, int x, int y) {
  param* p = &g_params[g_selected_param];
  int shift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
  int ctrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
  float mag = (shift ? 10 : 1) * (ctrl ? 0.1 : 1);
  switch (key) {
    case GLUT_KEY_UP:
      p->value = p->modulo ? ((int) (p->value + p->delta)) % p->modulo :
          p->value + p->delta*mag;
      p->value = p->value > p->max ? p->max : p->value;
      g_save_params("current.params");
      break;
    case GLUT_KEY_DOWN:
      p->value = p->modulo ?
          ((int) (p->value + p->modulo - p->delta)) % p->modulo :
          p->value - p->delta*mag;
      p->value = p->value < p->min ? p->min : p->value;
      g_save_params("current.params");
      break;
    case GLUT_KEY_LEFT:
      g_selected_param = (g_selected_param + g_num_params - 1) % g_num_params;
      break;
    case GLUT_KEY_RIGHT:
      g_selected_param = (g_selected_param + 1) % g_num_params;
      break;
  }
  g_show_params();
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
  glutSpecialFunc(g_special);
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
int f_count = 0;
frame* frames;
FILE* play_fp = NULL;
int f_heartbeat_count = 0;

#define TIMING_FRAMES 30
double frame_times[TIMING_FRAMES];
int f_time_i = 0;

double get_time() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec/1e6;
}

void f_depth_callback(freenect_device* dev, void* data, u32 timestamp) {
  int i, j;
  int x, y;
  int cam_rot_int = cam_rot;
  double now, interval;

  if (!f_paused) {
    pthread_mutex_lock(&depth_ready_mutex);
    switch (cam_rot_int) {
      case 0:
        for (i = 0; i < 640*480; i++) {
          f_depth[i] = depth_to_mm(((u16*) data)[i]);
        }
        break;
      case 1:
        bzero(f_depth, 640*480*sizeof(u16));
        for (x = 0; x < 480; x++) {
          for (y = 0; y < 480; y++) {
            i = y*640 + (x + 80);
            j = x*640 + (479 - y) + 80 + y_shift;
            f_depth[i] = depth_to_mm(((u16*) data)[j]);
          }
        }
        break;
      case 2:
        for (x = 0; x < 640; x++) {
          for (y = 0; y < 480; y++) {
            i = y*640 + x;
            j = (479 - y)*640 + (639 - x);
            f_depth[i] = depth_to_mm(((u16*) data)[j]);
          }
        }
        break;
      case 3:
        bzero(f_depth, 640*480*sizeof(u16));
        for (x = 0; x < 480; x++) {
          for (y = 0; y < 480; y++) {
            i = y*640 + (x + 80);
            j = (479 - x)*640 + y + 80 + y_shift;
            f_depth[i] = depth_to_mm(((u16*) data)[j]);
          }
        }
        break;
    }
    depth_ready = 1;
    pthread_cond_signal(&depth_ready_cond);
    pthread_mutex_unlock(&depth_ready_mutex);

    f_time_i = (f_time_i + 1) % TIMING_FRAMES;
    now = get_time();
    interval = now - frame_times[f_time_i];
    frame_times[f_time_i] = now;

    fprintf(stderr, "%5.1f fps / frame: %5d / particles: %3d \r",
            TIMING_FRAMES/interval, f_count, num_particles);
    f_count++;
  }
  if ((f_heartbeat_count++ & 31) == 0) close(creat("/tmp/heartbeat", 0644));
}

void* f_main(void* arg) {
  freenect_set_led(f_device, LED_OFF);
  freenect_set_depth_callback(f_device, f_depth_callback);
  freenect_set_depth_mode(f_device, freenect_find_depth_mode(
      FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
  freenect_start_depth(f_device);
  while (!f_should_quit && freenect_process_events(f_context) >= 0);
  freenect_stop_depth(f_device);
  freenect_close_device(f_device);
  freenect_shutdown(f_context);
  return NULL;
}

void* f_playback_main(void* arg) {
  int f, i;
  while (!f_should_quit) {
    for (f = 0; f < num_frames && !f_should_quit; f++) {
      f_count = f;
      f_depth_callback(NULL, frames[f].depth, 0);
      usleep(30000); 
      f -= f_paused;
    }
    f = num_frames - 1;
    for (i = 0; i < 60 && !f_should_quit; i++) {
      f_count = f;
      f_depth_callback(NULL, frames[f].depth, 0);
      i -= f_paused;
    }
  }
  fprintf(stderr, "\n");
  return NULL;
}

int main(int argc, char** argv) {
  FILE* fp;
  int r, c;

  fp = fopen("ranges.txt", "r");
  if (fp) {
    while (fscanf(fp, "%d %d\n",
                  &(g_pixel_ranges[g_num_pixel_ranges].start),
                  &(g_pixel_ranges[g_num_pixel_ranges].stop)) == 2) {
      g_num_pixel_ranges++;
    }
    fclose(fp);
  }

  fp = fopen("map.txt", "r");
  if (fp) {
    for (r = 0; r < rows; r++) {
      for (c = 0; c < cols; c++) {
        fscanf(fp, "%d", &(pixel_map[r][c]));
      }
      fscanf(fp, "\n");
    }
  }

  if (argc > 1) {
    g_sink = opc_new_sink(argv[1]);
    if (g_sink < 0) {
      fprintf(stderr, "Usage: %s <address>\n", argv[0]);
      exit(1);
    }
  }
  if (argc > 2) {
    if (strcmp(argv[2], "-") == 0) {
      play_fp = stdin;
    } else {
      play_fp = fopen(argv[2], "r");
      if (!play_fp) {
        fprintf(stderr, "Usage: %s <address> <filename>\n", argv[0]);
        exit(1);
      }
    }

    frames = malloc(MAX_FRAMES*sizeof(frame));
    for (num_frames = 0; num_frames < MAX_FRAMES; num_frames++) {
      if (fread(&(frames[num_frames]), sizeof(frame), 1, play_fp) == 0) {
        break;
      }
    }
    fprintf(stderr, "Read %d frame%s.\n",
            num_frames, num_frames == 1 ? "" : "s");
    pthread_create(&f_thread, NULL, f_playback_main, NULL);
  } else {
    // Open Freenect device 0.
    if (freenect_init(&f_context, NULL) < 0) {
      fprintf(stderr, "freenect_init failed\n");
      return 1;
    }
    freenect_set_log_level(f_context, FREENECT_LOG_DEBUG);
    freenect_select_subdevices(f_context, FREENECT_DEVICE_CAMERA);
    /*
#ifdef FREENECT_DEVICE_MOTOR
    freenect_select_subdevices(
        f_context, FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA);
#endif
    */
    if (freenect_open_device(f_context, &f_device, 0) < 0) {
      fprintf(stderr, "freenect_open_device failed\n");
      return 1;
    }
    pthread_create(&f_thread, NULL, f_main, NULL);
  }

  g_main(NULL); // Mac OS X requires GLUT to run on the main thread
  return 0;
}
