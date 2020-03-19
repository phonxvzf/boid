#include <iostream>
#include <cmath>
#include <ctime>
#include <limits>
#include <algorithm>

#include <SDL2/SDL_image.h>

#define BOID_RADIUS (8)
#define BOID_MAX_SPEED (300.f)
#define GRAVITY (200.f)
#define SCREEN_SIZE (1024)
#define FLOCK_RADIUS (100)
#define AVOID_RADIUS (50)

SDL_Window* window;
SDL_Renderer* renderer;

inline float pow2(float x) {
  return x * x;
}

float randf(bool allow_neg = false) {
  int i = std::rand();
  float val = 2.f * i / std::numeric_limits<uint32_t>::max();
  if (allow_neg) {
    i = std::rand();
    bool sign = 2.f * i / std::numeric_limits<uint32_t>::max() < 0.5f;
    if (sign) val *= -1.f;
  }
  return val;
}

class vec2 {
  public:
    float x, y;

    vec2(float x = 0, float y = 0) : x(x), y(y) {}
    vec2(const vec2& v) : x(v.x), y(v.y) {}

    vec2& operator=(const vec2& v) {
      x = v.x;
      y = v.y;
      return *this;
    }

    vec2 operator+(const vec2& v) const {
      return vec2(x + v.x, y + v.y);
    }

    vec2 operator+=(const vec2& v) {
      x += v.x;
      y += v.y;
      return *this;
    }

    vec2 operator-(const vec2& v) const {
      return vec2(x - v.x, y - v.y);
    }

    vec2 operator-=(const vec2& v) {
      x -= v.x;
      y -= v.y;
      return *this;
    }

    vec2 operator-() const {
      return vec2(-x, -y);
    }

    float dot(const vec2& v) const {
      return x * v.x + y * v.y;
    }

    float size() const {
      return std::sqrt(pow2(x) + pow2(y));
    }

    vec2 normalized() const {
      float sz = size();
      if (sz > 1e-6f) return vec2(x / sz, y / sz);
      return *this;
    }

    static inline float distance(const vec2& v0, const vec2& v1) {
      return std::sqrt(pow2(v0.x - v1.x) + pow2(v0.y - v1.y));
    }

    static inline float distance2(const vec2& v0, const vec2& v1) {
      return pow2(v0.x - v1.x) + pow2(v0.y - v1.y);
    }

    static vec2 sample(bool allow_neg = true) {
      return vec2(randf(allow_neg), randf(allow_neg));
    }
};

vec2 operator*(float s, const vec2& v) {
  return vec2(s * v.x, s * v.y);
}

vec2 operator*(const vec2& v, float s) {
  return s * v;
}

vec2 operator/(const vec2& v, float s) {
  return vec2(v.x / s, v.y / s);
}

class entity {
  public:
    vec2 position;
    vec2 velocity;
    vec2 accel;

    bool is_agent = false;

    entity(const vec2& p = 0, const vec2& v = 0, const vec2& a = 0)
      : position(p), velocity(v), accel(a) {}

    void move(float dt) {
      position += velocity * dt;
      if (position.x > SCREEN_SIZE - BOID_RADIUS || position.x < BOID_RADIUS) velocity.x *= -1.f;
      if (position.y > SCREEN_SIZE - BOID_RADIUS || position.y < BOID_RADIUS) velocity.y *= -1.f;
      velocity += accel * dt;
      velocity.x = std::min(velocity.x, BOID_MAX_SPEED);
      velocity.y = std::min(velocity.y, BOID_MAX_SPEED);
    }

    static bool overlap(const entity& e0, const entity& e1) {
      return vec2::distance2(e0.position, e1.position) <= pow2(2.f * BOID_RADIUS);
    }
};

class boid : public entity {
  public:
    int agent_id = -1;

    boid(const vec2& p = 0, const vec2& v = 0, const vec2& a = 0) : entity(p, v, a) {}
};

class agent : public entity {
  public:
    agent(const vec2& p = 0, const vec2& v = 0, const vec2& a = 0) : entity(p, v, a) {
      is_agent = true;
    }
};

class obstacle : public entity {
  public:
    obstacle(const vec2& p = 0) : entity(p) {}
};

void clean_after_init() {
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

double rad_to_deg(double rad) {
  return 180 / M_PI * rad;
}

double pow2(double x) {
  return x * x;
}

int main(int argc, char** argv) {
  std::srand(std::time(NULL));
  if (argc < 2) {
    std::cerr << "specify total number of boids"
      << std::endl;
    return 1;
  }

  int n_boids = atoi(argv[1]);
  int n_agents = 2;

  if (n_boids < 2)
    throw std::runtime_error("total number of boids should be greater than 2");

  if (SDL_Init(SDL_INIT_VIDEO)) {
    fprintf(stderr, "error: could not initialize SDL: %s\n", SDL_GetError());
    return 1;
  }

  if (IMG_Init(IMG_INIT_PNG) != IMG_INIT_PNG) {
    fprintf(stderr, "error: could not initialize SDL_image: %s\n", IMG_GetError());
    return 1;
  }

  window = SDL_CreateWindow(
      "boid", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_SIZE, SCREEN_SIZE, 0
      );
  renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
      );

  SDL_Surface* boid_surface = IMG_Load("boid.png");
  if (boid_surface == nullptr) throw std::runtime_error("error: could not load boid.png");
  SDL_Texture* boid_texture = SDL_CreateTextureFromSurface(renderer, boid_surface);
  SDL_FreeSurface(boid_surface);

  boid_surface = IMG_Load("agent.png");
  if (boid_surface == nullptr) throw std::runtime_error("error: could not load agent.png");
  SDL_Texture* agent_texture = SDL_CreateTextureFromSurface(renderer, boid_surface);
  SDL_FreeSurface(boid_surface);

  boid_surface = IMG_Load("obstacle.png");
  if (boid_surface == nullptr) throw std::runtime_error("error: could not load obstacle.png");
  SDL_Texture* obstacle_texture = SDL_CreateTextureFromSurface(renderer, boid_surface);
  SDL_FreeSurface(boid_surface);

  boid_surface = IMG_Load("a.png");
  if (boid_surface == nullptr) throw std::runtime_error("error: could not load a.png");
  SDL_Texture* a_texture = SDL_CreateTextureFromSurface(renderer, boid_surface);
  SDL_FreeSurface(boid_surface);

  boid_surface = IMG_Load("b.png");
  if (boid_surface == nullptr) throw std::runtime_error("error: could not load b.png");
  SDL_Texture* b_texture = SDL_CreateTextureFromSurface(renderer, boid_surface);
  SDL_FreeSurface(boid_surface);


  uint8_t running = true;
  SDL_Event e;
  float dt = 1e-3f;
  uint32_t tick = SDL_GetTicks();

  // generate entities
  int n_followers = n_boids - n_agents;

  boid boids[n_followers];
  agent agents[n_agents];
  obstacle obstacles[5];

  auto overlap = [&](const entity& e, bool only_obs) -> bool {
    for (int i = 0; i < 5; ++i) {
      if (&e == &obstacles[i]) continue;
      if (entity::overlap(e, obstacles[i])) return true;
    }
    if (only_obs) return false;
    for (int i = 0; i < n_followers; ++i) {
      if (&e == &boids[i]) continue;
      if (entity::overlap(e, boids[i])) return true;
    }
    for (int i = 0; i < n_agents; ++i) {
      if (&e == &agents[i]) continue;
      if (entity::overlap(e, agents[i])) return true;
    }
    return false;
  };

  auto near_friend = [&](const boid& b, float r) -> boid* {
    float r2 = pow2(r);
    for (int i = 0; i < n_followers; ++i) {
      if (&b == &boids[i]) continue;
      if (b.agent_id >= 0 && boids[i].agent_id == b.agent_id) {
        if (vec2::distance2(b.position, boids[i].position) <= r2) {
          return &boids[i];
        }
      }
    }
    return nullptr;
  };

  for (int i = 0; i < 5; ++i) {
    obstacles[i].position = vec2::sample(false) * SCREEN_SIZE;
  }

  for (int i = 0; i < n_followers; ++i) {
    do {
      boids[i].position = vec2::sample(false) * SCREEN_SIZE;
      boids[i].velocity = vec2::sample().normalized() * std::max(100.f, randf() * BOID_MAX_SPEED);
    } while (overlap(boids[i], false));
  }

  for (int i = 0; i < n_agents; ++i) {
    agents[i].velocity = vec2::sample().normalized() * std::max(100.f, randf() * BOID_MAX_SPEED);
  }

  const vec2 a_position(100.f, 100.f);
  const vec2 b_position(
      SCREEN_SIZE - 2 * BOID_RADIUS - 100.f, SCREEN_SIZE - 2 * BOID_RADIUS - 100.f
      );
  agents[0].position = b_position;
  agents[1].position = a_position;

  // main loop
  while (running) {
    tick = SDL_GetTicks();

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0xff);
    SDL_RenderClear(renderer);

    while (SDL_PollEvent(&e)) {
      switch (e.type) {
        case SDL_QUIT:
          running = false;
          break;
        case SDL_KEYDOWN:
          if (e.key.keysym.sym == SDLK_q) running = false;
          break;
      }
    }

    // update entities
    agents[0].accel = (a_position - agents[0].position).normalized() * GRAVITY;
    agents[1].accel = (b_position - agents[1].position).normalized() * GRAVITY;
    if (overlap(agents[0], true)) agents[0].velocity = -agents[0].velocity;
    if (overlap(agents[1], true)) agents[1].velocity = -agents[1].velocity;
    float dist2[2];
    for (int i = 0; i < n_followers; ++i) {
      dist2[0] = vec2::distance2(boids[i].position, agents[0].position);
      dist2[1] = vec2::distance2(boids[i].position, agents[1].position);
      boids[i].agent_id = 0;
      if (dist2[1] < dist2[0]) boids[i].agent_id = 1;
      boids[i].accel = (agents[boids[i].agent_id].position - boids[i].position).normalized()
        * GRAVITY * 3.5f;
      if (overlap(boids[i], true))
        boids[i].velocity = -boids[i].velocity;

      boid* friend_boid;
      if ((friend_boid = near_friend(boids[i], AVOID_RADIUS)) != nullptr) {
        boids[i].accel += (boids[i].position - friend_boid->position).normalized()
          * GRAVITY * 2.5f;
      }
    }

    // move entities
    for (int i = 0; i < n_followers; ++i) boids[i].move(dt);
    for (int i = 0; i < n_agents; ++i) agents[i].move(dt);

    // draw entities
    SDL_Rect r;
    r.w = BOID_RADIUS * 2;
    r.h = BOID_RADIUS * 2;
    for (int i = 0; i < n_followers; ++i) {
      r.x = boids[i].position.x - BOID_RADIUS;
      r.y = boids[i].position.y - BOID_RADIUS;
      SDL_RenderCopy(renderer, boid_texture, nullptr, &r);
    }
    for (int i = 0; i < n_agents; ++i) {
      r.x = agents[i].position.x - BOID_RADIUS;
      r.y = agents[i].position.y - BOID_RADIUS;
      SDL_RenderCopy(renderer, agent_texture, nullptr, &r);
    }
    for (int i = 0; i < 5; ++i) {
      r.x = obstacles[i].position.x - BOID_RADIUS;
      r.y = obstacles[i].position.y - BOID_RADIUS;
      SDL_RenderCopy(renderer, obstacle_texture, nullptr, &r);
    }
    r.x = a_position.x;
    r.y = a_position.y;
    SDL_RenderCopy(renderer, a_texture, nullptr, &r);
    r.x = b_position.x;
    r.y = b_position.y;
    SDL_RenderCopy(renderer, b_texture, nullptr, &r);

    SDL_RenderPresent(renderer);
    dt = (SDL_GetTicks() - tick) * 1e-3f;
  }

  SDL_DestroyTexture(boid_texture);
  SDL_DestroyTexture(agent_texture);
  SDL_DestroyTexture(obstacle_texture);
  SDL_DestroyTexture(a_texture);
  SDL_DestroyTexture(b_texture);

  clean_after_init();
  return 0;
}
