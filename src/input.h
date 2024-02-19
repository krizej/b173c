#ifndef B173C_INPUT_H
#define B173C_INPUT_H

#include "common.h"

#define KEYMOD_SHIFT    1 << 0
#define KEYMOD_CTRL     1 << 1
#define KEYMOD_ALT      1 << 2
#define KEYMOD_CAPSLOCK 1 << 3

void in_init(void);
void in_update(void);
void in_shutdown(void);

struct gamekeys {
    ubyte forward : 1;
    ubyte back    : 1;
    ubyte left    : 1;
    ubyte right   : 1;
    ubyte attack  : 1;
    ubyte attack2 : 1;
    ubyte jump    : 1;
    ubyte sneak   : 1;
};

struct key_status {
    bool pressed       : 1; // this is true while the key is pressed
    bool just_pressed  : 1; // this is only true for 1 frame
    bool just_released : 1; // this is only true for 1 frame
    bool echo : 1; // like just_pressed, except it allows for repetitions
    char *binding;          // the console command bound to this key
};

extern struct key_status input_keys[512]; // use KEY_* as the index

void key_bind(int key, char *bind);

int key_from_name(const char *name);
const char *name_from_key(int key);

/**
  THE FOLLOWING WAS COPIED FROM <SDL2/SDL_scancode.h> AND MODIFIED besides the
  license obviously
  ----------------------------------------------------------------------------------------------

  Simple DirectMedia Layer
  Copyright (C) 1997-2023 Sam Lantinga <slouken@libsdl.org>

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#define KEY_UNKNOWN 0

/**
 *  \name Usage page 0x07
 *
 *  These values are from usage page 0x07 (USB keyboard page).
 */
/* @{ */

#define KEY_A 4
#define KEY_B 5
#define KEY_C 6
#define KEY_D 7
#define KEY_E 8
#define KEY_F 9
#define KEY_G 10
#define KEY_H 11
#define KEY_I 12
#define KEY_J 13
#define KEY_K 14
#define KEY_L 15
#define KEY_M 16
#define KEY_N 17
#define KEY_O 18
#define KEY_P 19
#define KEY_Q 20
#define KEY_R 21
#define KEY_S 22
#define KEY_T 23
#define KEY_U 24
#define KEY_V 25
#define KEY_W 26
#define KEY_X 27
#define KEY_Y 28
#define KEY_Z 29

#define KEY_1 30
#define KEY_2 31
#define KEY_3 32
#define KEY_4 33
#define KEY_5 34
#define KEY_6 35
#define KEY_7 36
#define KEY_8 37
#define KEY_9 38
#define KEY_0 39

#define KEY_RETURN    40
#define KEY_ESCAPE    41
#define KEY_BACKSPACE 42
#define KEY_TAB       43
#define KEY_SPACE     44

#define KEY_MINUS        45
#define KEY_EQUALS       46
#define KEY_LEFTBRACKET  47
#define KEY_RIGHTBRACKET 48
#define KEY_BACKSLASH                                 \
    49 /**< Located at the lower left of the return   \
        *   key on ISO keyboards and at the right end \
        *   of the QWERTY row on ANSI keyboards.      \
        *   Produces REVERSE SOLIDUS (backslash) and  \
        *   VERTICAL LINE in a US layout REVERSE      \
        *   SOLIDUS and VERTICAL LINE in a UK Mac     \
        *   layout NUMBER SIGN and TILDE in a UK      \
        *   Windows layout DOLLAR SIGN and POUND SIGN \
        *   in a Swiss German layout NUMBER SIGN and  \
        *   APOSTROPHE in a German layout GRAVE       \
        *   ACCENT and POUND SIGN in a French Mac     \
        *   layout and ASTERISK and MICRO SIGN in a   \
        *   French Windows layout.                    \
        */
#define KEY_NONUSHASH                                  \
    50 /**< ISO USB keyboards actually use this code   \
        *   instead of 49 for the same key but all     \
        *   OSes I've seen treat the two codes         \
        *   identically. So as an implementor unless   \
        *   your keyboard generates both of those      \
        *   codes and your OS treats them differently  \
        *   you should generate SDL_SCANCODE_BACKSLASH \
        *   instead of this code. As a user you        \
        *   should not rely on this code because SDL   \
        *   will never generate it with most (all?)    \
        *   keyboards.                                 \
        */
#define KEY_SEMICOLON  51
#define KEY_APOSTROPHE 52
#define KEY_GRAVE                                         \
    53 /**< Located in the top left corner (on both ANSI  \
        *   and ISO keyboards). Produces GRAVE ACCENT and \
        *   TILDE in a US Windows layout and in US and UK \
        *   Mac layouts on ANSI keyboards GRAVE ACCENT    \
        *   and NOT SIGN in a UK Windows layout SECTION   \
        *   SIGN and PLUS-MINUS SIGN in US and UK Mac     \
        *   layouts on ISO keyboards SECTION SIGN and     \
        *   DEGREE SIGN in a Swiss German layout (Mac:    \
        *   only on ISO keyboards) CIRCUMFLEX ACCENT and  \
        *   DEGREE SIGN in a German layout (Mac: only on  \
        *   ISO keyboards) SUPERSCRIPT TWO and TILDE in a \
        *   French Windows layout COMMERCIAL AT and       \
        *   NUMBER SIGN in a French Mac layout on ISO     \
        *   keyboards and LESS-THAN SIGN and GREATER-THAN \
        *   SIGN in a Swiss German German or French Mac   \
        *   layout on ANSI keyboards.                     \
        */
#define KEY_COMMA  54
#define KEY_PERIOD 55
#define KEY_SLASH  56

#define KEY_CAPSLOCK 57

#define KEY_F1  58
#define KEY_F2  59
#define KEY_F3  60
#define KEY_F4  61
#define KEY_F5  62
#define KEY_F6  63
#define KEY_F7  64
#define KEY_F8  65
#define KEY_F9  66
#define KEY_F10 67
#define KEY_F11 68
#define KEY_F12 69

#define KEY_PRINTSCREEN 70
#define KEY_SCROLLLOCK  71
#define KEY_PAUSE       72
#define KEY_INSERT                                       \
    73 /**< insert on PC help on some Mac keyboards (but \
                                   does send code 73 not 117) */
#define KEY_HOME     74
#define KEY_PAGEUP   75
#define KEY_DELETE   76
#define KEY_END      77
#define KEY_PAGEDOWN 78
#define KEY_RIGHT    79
#define KEY_LEFT     80
#define KEY_DOWN     81
#define KEY_UP       82

#define KEY_NUMLOCKCLEAR                          \
    83 /**< num lock on PC clear on Mac keyboards \
        */
#define KEY_KP_DIVIDE   84
#define KEY_KP_MULTIPLY 85
#define KEY_KP_MINUS    86
#define KEY_KP_PLUS     87
#define KEY_KP_ENTER    88
#define KEY_KP_1        89
#define KEY_KP_2        90
#define KEY_KP_3        91
#define KEY_KP_4        92
#define KEY_KP_5        93
#define KEY_KP_6        94
#define KEY_KP_7        95
#define KEY_KP_8        96
#define KEY_KP_9        97
#define KEY_KP_0        98
#define KEY_KP_PERIOD   99

#define KEY_NONUSBACKSLASH                        \
    100                     /**< This is the additional key that ISO  \
         *   keyboards have over ANSI ones        \
         *   located between left shift and Y.    \
         *   Produces GRAVE ACCENT and TILDE in a \
         *   US or UK Mac layout REVERSE SOLIDUS  \
         *   (backslash) and VERTICAL LINE in a   \
         *   US or UK Windows layout and          \
         *   LESS-THAN SIGN and GREATER-THAN SIGN \
         *   in a Swiss German German or French   \
         *   layout. */
#define KEY_APPLICATION 101 /**< windows contextual menu compose */
#define KEY_POWER                                        \
    102 /**< The USB document says this is a status flag \
         *   not a physical key - but some Mac keyboards \
         *   do have a power key. */
#define KEY_KP_EQUALS      103
#define KEY_F13            104
#define KEY_F14            105
#define KEY_F15            106
#define KEY_F16            107
#define KEY_F17            108
#define KEY_F18            109
#define KEY_F19            110
#define KEY_F20            111
#define KEY_F21            112
#define KEY_F22            113
#define KEY_F23            114
#define KEY_F24            115
#define KEY_EXECUTE        116
#define KEY_HELP           117 /**< AL Integrated Help Center */
#define KEY_MENU           118 /**< Menu (show menu) */
#define KEY_SELECT         119
#define KEY_STOP           120 /**< AC Stop */
#define KEY_AGAIN          121 /**< AC Redo/Repeat */
#define KEY_UNDO           122 /**< AC Undo */
#define KEY_CUT            123 /**< AC Cut */
#define KEY_COPY           124 /**< AC Copy */
#define KEY_PASTE          125 /**< AC Paste */
#define KEY_FIND           126 /**< AC Find */
#define KEY_MUTE           127
#define KEY_VOLUMEUP       128
#define KEY_VOLUMEDOWN     129
/* not sure whether there's a reason to enable these */
/*     SDL_SCANCODE_LOCKINGCAPSLOCK 130  */
/*     SDL_SCANCODE_LOCKINGNUMLOCK 131 */
/*     SDL_SCANCODE_LOCKINGSCROLLLOCK 132 */
#define KEY_KP_COMMA       133
#define KEY_KP_EQUALSAS400 134

#define KEY_INTERNATIONAL1               \
    135 /**< used on Asian keyboards see \
                                            footnotes in USB doc */
#define KEY_INTERNATIONAL2 136
#define KEY_INTERNATIONAL3 137 /**< Yen */
#define KEY_INTERNATIONAL4 138
#define KEY_INTERNATIONAL5 139
#define KEY_INTERNATIONAL6 140
#define KEY_INTERNATIONAL7 141
#define KEY_INTERNATIONAL8 142
#define KEY_INTERNATIONAL9 143
#define KEY_LANG1          144 /**< Hangul/English toggle */
#define KEY_LANG2          145 /**< Hanja conversion */
#define KEY_LANG3          146 /**< Katakana */
#define KEY_LANG4          147 /**< Hiragana */
#define KEY_LANG5          148 /**< Zenkaku/Hankaku */
#define KEY_LANG6          149 /**< reserved */
#define KEY_LANG7          150 /**< reserved */
#define KEY_LANG8          151 /**< reserved */
#define KEY_LANG9          152 /**< reserved */

#define KEY_ALTERASE   153 /**< Erase-Eaze */
#define KEY_SYSREQ     154
#define KEY_CANCEL     155 /**< AC Cancel */
#define KEY_CLEAR      156
#define KEY_PRIOR      157
#define KEY_RETURN2    158
#define KEY_SEPARATOR  159
#define KEY_OUT        160
#define KEY_OPER       161
#define KEY_CLEARAGAIN 162
#define KEY_CRSEL      163
#define KEY_EXSEL      164

#define KEY_KP_00              176
#define KEY_KP_000             177
#define KEY_THOUSANDSSEPARATOR 178
#define KEY_DECIMALSEPARATOR   179
#define KEY_CURRENCYUNIT       180
#define KEY_CURRENCYSUBUNIT    181
#define KEY_KP_LEFTPAREN       182
#define KEY_KP_RIGHTPAREN      183
#define KEY_KP_LEFTBRACE       184
#define KEY_KP_RIGHTBRACE      185
#define KEY_KP_TAB             186
#define KEY_KP_BACKSPACE       187
#define KEY_KP_A               188
#define KEY_KP_B               189
#define KEY_KP_C               190
#define KEY_KP_D               191
#define KEY_KP_E               192
#define KEY_KP_F               193
#define KEY_KP_XOR             194
#define KEY_KP_POWER           195
#define KEY_KP_PERCENT         196
#define KEY_KP_LESS            197
#define KEY_KP_GREATER         198
#define KEY_KP_AMPERSAND       199
#define KEY_KP_DBLAMPERSAND    200
#define KEY_KP_VERTICALBAR     201
#define KEY_KP_DBLVERTICALBAR  202
#define KEY_KP_COLON           203
#define KEY_KP_HASH            204
#define KEY_KP_SPACE           205
#define KEY_KP_AT              206
#define KEY_KP_EXCLAM          207
#define KEY_KP_MEMSTORE        208
#define KEY_KP_MEMRECALL       209
#define KEY_KP_MEMCLEAR        210
#define KEY_KP_MEMADD          211
#define KEY_KP_MEMSUBTRACT     212
#define KEY_KP_MEMMULTIPLY     213
#define KEY_KP_MEMDIVIDE       214
#define KEY_KP_PLUSMINUS       215
#define KEY_KP_CLEAR           216
#define KEY_KP_CLEARENTRY      217
#define KEY_KP_BINARY          218
#define KEY_KP_OCTAL           219
#define KEY_KP_DECIMAL         220
#define KEY_KP_HEXADECIMAL     221

#define KEY_LCTRL  224
#define KEY_LSHIFT 225
#define KEY_LALT   226 /**< alt option */
#define KEY_LGUI   227 /**< windows command (apple) meta */
#define KEY_RCTRL  228
#define KEY_RSHIFT 229
#define KEY_RALT   230 /**< alt gr option */
#define KEY_RGUI   231 /**< windows command (apple) meta */

#define KEY_MODE                                         \
    257 /**< I'm not sure if this is really not covered  \
         *   by any of the above but since there's a     \
         *   special KMOD_MODE for it I'm adding it here \
         */

/* @} */ /* Usage page 0x07 */

/**
 *  \name Usage page 0x0C
 *
 *  These values are mapped from usage page 0x0C (USB consumer page).
 *  See https://usb.org/sites/default/files/hut1_2.pdf
 *
 *  There are way more keys in the spec than we can represent in the
 *  current scancode range so pick the ones that commonly come up in
 *  real world usage.
 */
/* @{ */

#define KEY_AUDIONEXT    258
#define KEY_AUDIOPREV    259
#define KEY_AUDIOSTOP    260
#define KEY_AUDIOPLAY    261
#define KEY_AUDIOMUTE    262
#define KEY_MEDIASELECT  263
#define KEY_WWW          264 /**< AL Internet Browser */
#define KEY_MAIL         265
#define KEY_CALCULATOR   266 /**< AL Calculator */
#define KEY_COMPUTER     267
#define KEY_AC_SEARCH    268 /**< AC Search */
#define KEY_AC_HOME      269 /**< AC Home */
#define KEY_AC_BACK      270 /**< AC Back */
#define KEY_AC_FORWARD   271 /**< AC Forward */
#define KEY_AC_STOP      272 /**< AC Stop */
#define KEY_AC_REFRESH   273 /**< AC Refresh */
#define KEY_AC_BOOKMARKS 274 /**< AC Bookmarks */

/* @} */ /* Usage page 0x0C */

/**
 *  \name Walther keys
 *
 *  These are values that Christian Walther added (for mac keyboard?).
 */
/* @{ */

#define KEY_BRIGHTNESSDOWN 275
#define KEY_BRIGHTNESSUP   276
#define KEY_DISPLAYSWITCH                   \
    277 /**< display mirroring/dual display \
                                           switch video mode switch */
#define KEY_KBDILLUMTOGGLE 278
#define KEY_KBDILLUMDOWN   279
#define KEY_KBDILLUMUP     280
#define KEY_EJECT          281
#define KEY_SLEEP          282 /**< SC System Sleep */

#define KEY_APP1 283
#define KEY_APP2 284

/* @} */ /* Walther keys */

/**
 *  \name Usage page 0x0C (additional media keys)
 *
 *  These values are mapped from usage page 0x0C (USB consumer page).
 */
/* @{ */

#define KEY_AUDIOREWIND      285
#define KEY_AUDIOFASTFORWARD 286

/* @} */ /* Usage page 0x0C (additional media keys) */

/**
 *  \name Mobile keys
 *
 *  These are values that are often used on mobile phones.
 */
/* @{ */

#define KEY_SOFTLEFT                                                       \
    287 /**< Usually situated below the display on phones and              \
                                      used as a multi-function feature key \
           for selecting a software defined function shown on the bottom   \
           left of the display. */
#define KEY_SOFTRIGHT                                                        \
    288 /**< Usually situated below the display on phones and                \
                       used as a multi-function feature key for selecting    \
                       a software defined function shown on the bottom right \
                       of the display. */
#define KEY_CALL    289 /**< Used for accepting phone calls. */
#define KEY_ENDCALL 290 /**< Used for rejecting phone calls. */

#define KEY_MOUSE1         507
#define KEY_MOUSE2         508
#define KEY_MOUSE3         509
#define KEY_MOUSEWHEELUP   510
#define KEY_MOUSEWHEELDOWN 511

#define KEY_NUM 512

#endif
