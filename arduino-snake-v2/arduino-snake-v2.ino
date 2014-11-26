#include <IRremote.h>

// the pin to receive infrared signals
#define INFRARED_REMOTE_PIN     (A0)

// max length of snake's body, head is excluded.
#define SNAKE_MAX_LENGTH        (16)

// time span for snake's movement
#define SNAKE_MOVE_SPAN         (500)

// time span to spawn new fruit for snake
#define FRUIT_SPAWN_SPAN        (3000)

// time span to receive next code from remote control
#define REMOTE_RECEIVE_SPAN     (300)

// remote code of four different direction
// (0, 0) is in the right-bottom corner of a 8x8 LED matrix.
#define REMOTE_UP               (0x85AEFD19)
#define REMOTE_DOWN             (0xB66D73CD)
#define REMOTE_RIGHT            (0x5C5C901D)
#define REMOTE_LEFT             (0x8F890759)

//------------------------------------------------------------------------------
struct Position {
  int   x;
  int   y;

  bool operator== (const Position& that) const {
    return this->x == that.x && this->y == that.y;
  }
};

//------------------------------------------------------------------------------
class SnakeBoard {
  // game state: menu -> play -> over -> menu -> ......
  enum {
    STATE_MENU,
    STATE_PLAY,
    STATE_OVER,
  };

  enum {
    DIRECTION_NONE = -1,
    DIRECTION_UP = 0,
    DIRECTION_RIGHT,
    DIRECTION_DOWN,
    DIRECTION_LEFT,
  };

  enum {
    PIN_SIFTING_OUT_LATCH = 8,
    PIN_SIFTING_CLOCK = 11,
    PIN_SIFTING_DATA = 10,
  };

  // output pins of auduino for 8x8 LED matrix.
  static const int      ROW[8];

  // a dots image to diaplay when a game is finished.
  static const Position GAMEOVER_SCREEN[10];

  // a mask to mark bright LEDs
  // one integer for each row, one bit for each colum in a row.
  int       masks[8];

  // body of a snake, each element store the direction from previous dot to
  // current dot. The 2nd dot is in the direction of 'snake_body[0]' of the
  // snake's head
  int       snake_body[SNAKE_MAX_LENGTH];

  // current snake's length without head
  int       snake_length;

  // position of snake's head
  Position  snake_head;

  // position of new fruit for the snake.
  // fruit.x less than 0 means there is no fruit.
  Position  fruit;

  // game state: menu -> play -> over -> menu -> ......
  int     state;

  // next time the snake's will move to current_direction from snake_head.
  int     current_direction;

  // time span to update snake's movement
  long    snake_move_span;

  // time span to spawn new fruit
  long    fruit_spawn_span;

  // remote interface
  IRrecv          remote;
  decode_results  remote_results;

  // for remote singals repetition filtering
  long            remote_previous_time;

protected:
  // from pos, go to direction, the new position is des
  // return false if there is no valid destination (collide edges)
  static bool Destination(Position& des, const Position& pos, int direction) {
    bool ok = true;

    des = pos;

    switch (direction) {
      case SnakeBoard::DIRECTION_UP: {
        if (des.y < 7) {
          des.y += 1;
        } else {
          ok = false;
        }
      } break;
      case SnakeBoard::DIRECTION_DOWN: {
        if (des.y > 0) {
          des.y -= 1;
        } else {
          ok = false;
        }
      } break;
      case SnakeBoard::DIRECTION_LEFT: {
        if (des.x < 7) {
          des.x += 1;
        } else {
          ok = false;
        }
      } break;
      case SnakeBoard::DIRECTION_RIGHT: {
        if (des.x > 0) {
          des.x -= 1;
        } else {
          ok = false;
        }
      } break;
      default:
        break;
    }

    return ok;
  }

  // update pos to its neighbor in direction.
  // return false if it collide a edge.
  static bool Destination(Position& pos, int direction) {
    Position tmp = pos;

    return SnakeBoard::Destination(pos, tmp, direction);
  }

  // return inverse direction of input direction
  static int Invert(int direction) {
    return (direction + 2) % 4;
  }

  // check if the position on mask is not 0.
  bool Occupied(const Position& pos) {
    return 0 != (this->masks[pos.y] & (1 << pos.x));
  }

  // lit or de-lit the LED in pos
  void Lit(bool on, const Position& pos) {
    int m = this->masks[pos.y];

    if (on) {
      m |= (1 << pos.x);
    } else {
      m &= ~(1 << pos.x);
    }

    this->masks[pos.y] = m;
  }

  // go to menu
  void Reset() {
    this->state = SnakeBoard::STATE_MENU;

    this->remote_previous_time = millis();

    this->snake_length = 0;

    this->snake_head.x = 1;
    this->snake_head.y = 1;

    this->fruit.x = -1;
    this->fruit.y = -1;

    this->current_direction = DIRECTION_NONE;

    for (int i = 0; i < 8; ++i) {
      this->masks[i] = 0;
    }

    this->Lit(true, this->snake_head);
  }

  // receive infrared and change 'current direction'
  void Receive() {
    //--keep receiving remote data
    if (this->remote.decode(&this->remote_results)) {
      long remote_current_time = millis();

      if (this->remote_previous_time + REMOTE_RECEIVE_SPAN <= remote_current_time) {
        this->remote_previous_time = remote_current_time;

        switch (this->remote_results.value) {
          case REMOTE_UP :    this->current_direction = DIRECTION_UP;     break;
          case REMOTE_RIGHT : this->current_direction = DIRECTION_RIGHT;  break;
          case REMOTE_DOWN :  this->current_direction = DIRECTION_DOWN;   break;
          case REMOTE_LEFT :  this->current_direction = DIRECTION_LEFT;   break;
        default: break;
        }
      }

      //--Receive the next value
      this->remote.resume();
    }
  }

  // spawn new fruit
  void SpawnFruit(long current_time) {
    // spawn new fruit only when there is no fruit.
    if (this->fruit.x < 0) {
      // spawn new fruit after FRUIT_SPAWN_SPAN milliseconds when previous one
      // has be collected.
      if (this->fruit_spawn_span + FRUIT_SPAWN_SPAN <= current_time) {
        // candidates = all - head - body
        int count_empty = 64 - 1 - this->snake_length;

        // 1 is dummy
        int target_index = 1 + current_time % count_empty;

        int row, col, msk;

        for (row = 0; row < 8; ++row) {
          msk = this->masks[row];

          for (col = 0; col < 8; ++col) {
            if (0 == (msk & (1 << col))) {
              target_index -= 1;

              // found
              if (0 == target_index) {
                this->fruit.x = col;
                this->fruit.y = row;

                this->Lit(true, this->fruit);

                return;
              }
            }
          }
        }
      }
    }
  }

  // go to game over state
  void GameOver() {
    for (int i = 0; i < 8; ++i) {
      this->masks[i] = 0;
    }

    this->state = STATE_OVER;
    this->current_direction = DIRECTION_NONE;
  }

  // update contents of menu state
  void UpdateMenu() {
    if (this->current_direction == DIRECTION_NONE) {
      // blink the dots that represent snake's head

      this->Lit((millis() / 100) % 10 < 8, this->snake_head);
    } else {
      // direction is changed because user click some button on remote control.

      this->Lit(true, this->snake_head);

      this->state = STATE_PLAY;
      this->snake_move_span = millis();
      this->fruit_spawn_span = this->snake_move_span;
    }
  }

  // game play
  void UpdatePlay() {
    long current_time = millis();

    // snake move on step every SNAKE_MOVE_SPAN milliseconds
    if (this->snake_move_span + SNAKE_MOVE_SPAN > current_time) {
      return;
    }

    this->snake_move_span = current_time;

    Position dest;

    if (!SnakeBoard::Destination(dest, this->snake_head, this->current_direction)) {
      //--collided with a edge, game over!

      this->GameOver();

      return;
    }

    if (dest == this->fruit) {
      // there is a fruit in destination
      // if the snake is still hungary
      if (this->snake_length < SNAKE_MAX_LENGTH) {
        // merge the fruit as new snake head
        for (int i = this->snake_length; i > 0; --i) {
          this->snake_body[i] = this->snake_body[i - 1];
        }

        this->snake_body[0] = SnakeBoard::Invert(this->current_direction);

        this->snake_length += 1;
        this->snake_head = dest;
      } else {
        // the snake is full, eat the fruit with increasing body length

        int dir_prev = SnakeBoard::Invert(this->current_direction);
        int dir_next;

        Position tail = this->snake_head;

        for (int i = 0; i < this->snake_length; ++i) {
          dir_next = this->snake_body[i];

          this->snake_body[i] = dir_prev;

          dir_prev = dir_next;

          SnakeBoard::Destination(tail, dir_next);
        }

        this->snake_head = dest;
      }

      //--remove fruit
      this->fruit.x = -1;
      this->fruit.y = -1;

      //--next fruit will spawn after FRUIT_SPAWN_SPAN milliseconds
      this->fruit_spawn_span = current_time;
    } else if (this->Occupied(dest)) {
      // if there is something in destination (not fruit), the snake collide his
      // own body, game over
      this->GameOver();
      return;
    } else {
      // snake go !!!
      this->Lit(true, dest);

      int dir_prev = SnakeBoard::Invert(this->current_direction);
      int dir_next;

      Position tail = this->snake_head;

      for (int i = 0; i < this->snake_length; ++i) {
        dir_next = this->snake_body[i];

        this->snake_body[i] = dir_prev;

        dir_prev = dir_next;

        SnakeBoard::Destination(tail, dir_next);
      }

      this->Lit(false, tail);

      this->snake_head = dest;
    }

    this->SpawnFruit(current_time);
  }

  void UpdateOver() {
    if (this->current_direction == DIRECTION_NONE) {
      // blink the gameover screen
      for (int i = 0; i < 8; ++i) {
        this->masks[i] = 0;
      }

      if ((millis() / 100) % 10 < 8) {
        for (int i = sizeof(SnakeBoard::GAMEOVER_SCREEN) / sizeof(Position); i >= 0; --i) {
          this->Lit(true, SnakeBoard::GAMEOVER_SCREEN[i]);
        }
      }
    } else {
      // go to menu
      this->Reset();
    }
  }

  void Update() {
    switch(this->state) {
      case STATE_MENU:  this->UpdateMenu(); break;
      case STATE_PLAY:  this->UpdatePlay(); break;
      case STATE_OVER:  this->UpdateOver(); break;
      default:          this->UpdateOver(); break;
    }
  }

  // update LEDs base on masks
  void Refresh() {
    for (int row = 0; row < 8; ++row) {
      digitalWrite(SnakeBoard::ROW[row], HIGH);

      digitalWrite(PIN_SIFTING_OUT_LATCH, LOW);
      shiftOut(
        PIN_SIFTING_DATA,
        PIN_SIFTING_CLOCK,
        LSBFIRST,
        ~(unsigned char)this->masks[row]);
      digitalWrite(PIN_SIFTING_OUT_LATCH, HIGH);

      digitalWrite(PIN_SIFTING_OUT_LATCH, LOW);
      shiftOut(PIN_SIFTING_DATA, PIN_SIFTING_CLOCK, MSBFIRST, 255);
      digitalWrite(PIN_SIFTING_OUT_LATCH, HIGH);

      digitalWrite(ROW[row], LOW);
    }
  }

public:
  SnakeBoard() : snake_length(0), remote(INFRARED_REMOTE_PIN) {
  }

  void Setup() {
    for (int i = 0; i < 8; ++i) {
      pinMode(SnakeBoard::ROW[i], OUTPUT);

      digitalWrite(SnakeBoard::ROW[i], HIGH);
    }

    pinMode(PIN_SIFTING_OUT_LATCH, OUTPUT);
    pinMode(PIN_SIFTING_CLOCK, OUTPUT);
    pinMode(PIN_SIFTING_DATA, OUTPUT);

    this->remote.enableIRIn();

    this->Reset();
  }

  void Loop() {
    this->Receive();
    this->Update();
    this->Refresh();
  }
};

const int SnakeBoard::ROW[8] = {
  2, 7, 19, 5, 13, 18, 12, 16,
};

const Position SnakeBoard::GAMEOVER_SCREEN[10] = {
  {7, 5}, {6, 6}, {5, 6},
  {2, 6}, {1, 6}, {0, 5},
  {5, 1}, {4, 2}, {3, 2}, {2, 1}
};

SnakeBoard g_board;

void setup() {
  g_board.Setup();
}

void loop() {
  g_board.Loop();
}
