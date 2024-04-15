#ifndef _melodia_def_H
#define _melodia_def_H
#include "main.h"
/*Octave 2*/
#define C2   65
#define Db2  69
#define D2   73
#define Eb2  78
#define E2   82
#define F2   87
#define Gb2  93
#define G2   98
#define Ab2  104
#define A2   110
#define Bb2  117
#define B2   124
/*Octave 3*/
#define C3   131
#define Db3  139
#define D3   147
#define Eb3  156
#define E3   165
#define F3   175
#define Gb3  185
#define G3   196
#define Ab3  208
#define A3   220
#define Bb3  233
#define B3   247
/*Octave 4*/
#define C4   262
#define Db4  277
#define D4   294
#define Eb4  311
#define E4   330
#define F4   349
#define Gb4  370
#define G4   392
#define Ab4  415
#define A4   440
#define Bb4  466
#define B4   494
/*Octave 5*/
#define C5   523
#define Db5  554
#define D5   587
#define Eb5  622
#define E5   659
#define F5   698
#define Gb5  740
#define G5   784
#define Ab5  831
#define A5   880
#define Bb5  932
#define B5   988
/*Octave 6*/
#define C6   1047
#define Db6  1109
#define D6   1175
#define Eb6  1245
#define E6   1319
#define F6   1397
#define Gb6  1480
#define G6   1568
#define Ab6  1661
#define A6   1760
#define Bb6  1865
#define B6   1976


const uint8_t note_basic[]={1,2,3,4,5,6,7,1+7,2+7,3+7,4+7,5+7,6+7,7+7,1+14,2+14,3+14,4+14,5+14,6+14,7+14};
const uint8_t note_happy_song[]={5,5,6,5,1+7,7,0,5,5,6,5,2+7,1+7,5,5,5+7,3+7,1+7,7,6,0,4+7,4+7,3+7,1+7,2+7,1+7 };
const uint16_t beat_happy_song[]={250,250,500,500,500,500,500,250,250,500,500,500,1000,
                            250,250,500,500,500,500,1500,500,250,250,500,500,500,1000};
const uint8_t note_little_star[]={1,1,5,5,6,6,5,4,4,3,3,2,2,1,5,5,4,4,3,3,2,5,5,4,4,3,3,2,1,1,5,5,6,6,5,4,4,3,3,2,2,1};
const uint16_t beat_little_star[]={500,500,500,500,500,500,1000,500,500,500,500,500,500,1000,500,500,500,500,500,500,
																		1000,500,500,500,500,500,500,1000,500,500,500,500,500,500,1000,500,500,500,500,500,500,1000};
const uint8_t  note_little_bee[]={5,3,3,4,2,2,1,2,3,4,5,5,5,5,3,3,4,2,2,1,3,5,5,3,
																2,2,2,2,2,3,4,3,3,3,3,3,4,5,5,3,3,4,2,2,1,3,5,5,1};	
		
const uint16_t beat_little_bee[]={250,250,500,250,250,500,250,250,250,250,250,250,500,250,250,500,250,250,500,250,250,250,250,1000,
															250,250,250,250,250,250,500,250,250,250,250,250,250,500,250,250,500,250,250,500,250,250,250,250,1000};	


uint32_t Melody_of_Megalovania [] = {
    //DOEH DEH DEH AH DAH DOOEH DOO AH
    D4, D4, D5, A4, Ab4, G4, F4, D4, F4, G4,
    C4, C4, C4, C4,
    D5, A4, Ab4, G4, F4, D4, F4, G4, B3, B3,
    D5, A4, Ab4, G4, F4, D4, F4, G4, Bb3, Bb3,
    Bb3, Bb3, D5, A4, Ab4, G4, F4, D4, F4, G4,

    D4, D4, D5, A4, Ab4, G4, F4, D4, F4, G4,
    C4, C4, C4, C4,
    D5, A4, Ab4, G4, F4, D4, F4, G4, B3, B3,
    D5, A4, Ab4, G4, F4, D4, F4, G4, Bb3, Bb3,
    Bb3, Bb3, D5, A4, Ab4, G4, F4, D4, F4, G4,

    //DOEH DEH DEH AH DAH DOOEH DOO AH  (INTENSIFIES)
    D5, D5, D6, A5, Ab5, G5, F5, D5, F5, G5, 
    C5, C5, D6, A5, Ab5, G5, F5, D5, F5, G5,
    B4, B4, D6, A5, Ab5, G5, F5, D5, F5, G5,
    Bb4, Bb4, D6, A5, Ab5, G5, F5, D5, F5, G5,

    D5, D5, D6, A5, Ab5, G5, F5, D5, F5, G5,
    C5, C5, D6, A5, Ab5, G5, F5, D5, F5, G5,
    B4, B4, D6, A5, Ab5, G5, F5, D5, F5, G5,
    Bb4, Bb4, D6, A5, Ab5, G5, F5, D5, F5, G5,

    //DU DU DUDU DU DU DU
    F5, F5, F5, F5, F5, D5, D5, F5, F5, F5,
    G5, Ab5, G5, Ab5, G5, F5, D5, F5, G5, F5,
    F5, F5, G5, Ab5, A5, C6, A5, D6, D6, D6,
    A5, D6, C6, G6,

    //DU DU DUDU DU DU DUU  (INTENSIFIES)
    A5, A5, A5, A5, A5, G5, G5, A5, A5, A5,
    A5, G5, A5, D6, A5, G5, D6, A5, G5, F5, 
    C6, G5, F5, E5, Bb4, C5, D5, F5, C6,

    //Epic part
    F5, D5, F5, G5, Ab5, G5, F5, D5, Ab5, G5,
    F5, D5, F5, G5, Ab5, A5, C6, A5, Ab5, G5,
    F5, D5, E5, F5, G5, A5, C6, Db6, Ab5, Ab5, 
    G5, F5, G5, F4, G4, A4, F5, E5, D5, E5, 
    F5, G5, E5, A5, A5, Ab5, G5, Gb5, F5, E5,
    Eb5, D5, Db5, Eb5, F5, D5, F5, G5, Ab5, G5,
    F5, D5, Ab5, G5, F5, D5, F5, G5, Ab5, A5, 
    C6, A5, Ab5, G5, F5, D5, E5, F5, G5, A5,
    C6, Db6, Ab5, Ab5, G5, F5, G5, F4, G4, A4,
    F5, E5, D5, E5, F5, G5, E5, A5, A5, Ab5,
    G5, Gb5, F5, E5, Eb5, D5, Db5, Eb5,

    //Opera
    Bb3, F4, E4, D4, F4, Bb3, F4, E4, D4, D4,
    D4, Db4, C4,  B3, Bb3, A3, Ab3, G3, Gb3, F3,
    E3, Eb3, D3, Bb3, F4, E4, D4, F4, B2, G3, 
    F4, D4, G4, F4, C4, D4, C4, A3, G3, C4,     
    Bb3, F4, E4, D4, D3, D3, F4, E4, C4, E4,
    B3, G3, A3, C4, D3, D3, F4, E4, C4, E4,
    B3, G3, A3, C4,

    //Rock part
    Bb3, Bb3, Bb2, Bb3, Bb3, Bb3, Bb2, Bb2, Bb2, Bb3,
    C4, C4, C3, C4, C4, C4, C3, C3, C3, C4,    
    D4, D4, D3, D4, Db4, Db4, Db3, Db3, Db3, Db4,
    C4, C4, C3, C4, B3, B3, B2, B2, B2, B3,
    Bb3, Bb3, Bb2, Bb3, Bb3, Bb3, Bb2, Bb2, Bb2, Bb3, 
    C4, C4, C3, C4, C4, C4, C3, C3, C3, C4, 
    D4, D4, D3, D4, D4, D4, D3, D3, D3, D4,
    D4, D4, D3, D4, D4, D4, D3, D3, D3, D4,
    Bb3, Bb3, Bb2, Bb3, Bb3, Bb3, Bb2, Bb2, Bb2, Bb3,
    C4, C4, C3, C4, C4, C4, C3, C3, C3, C4,
    D4, D4, D3, D4, Db4, Db4, Db3, Db3, Db3, Db4,
    C4, C4, C3, C4, B3, B3, B2, B2, B2, B3,
    Bb3, Bb3, Bb2, Bb3, Bb3, Bb3, Bb2, Bb2, Bb2, Bb3,
    C4, C4, C3, C4, C4, C4, C3, C3, C3, C4,
    D4, D3, D4, A3, D4, D4, D4, F3, D3, D4,
    G3, D4, D3, D4, A3, D4, D4, D4, F3, D3,
    D4, G3,

    //End part
    D4, D4, D5, A4, Ab4, G4, F4, D4, F4, G4,
    C4, C4, C4, C4,
    D5, A4, Ab4, G4, F4, D4, F4, G4, B3, B3, 
    D5, A4, Ab4, G4, F4, D4, F4, G4, Bb3, Bb3,
    Bb3, Bb3, D5, A4, Ab4, G4, F4, D4, F4, G4,
    D4, D4, D5, A4, Ab4, G4, F4, D4, F4, G4,
    C4, C4, C4, C4,
    D5, A4, Ab4, G4, F4, D4, F4, G4,
};

uint32_t duration_of_Megalovania[] = {// (ms)
    //DOEH DEH DEH AH DAH DOOEH DOO AH
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 125, 125,
    250, 375, 125, 250, 250, 125, 125, 125, 125, 125,
    250, 375, 125, 250, 250, 125, 125, 125, 62, 62, 
    62, 62, 250, 375, 125, 250, 250, 125, 125, 125,

    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 125, 125,
    250, 375, 125, 250, 250, 125, 125, 125, 125, 125,
    250, 375, 125, 250, 250, 125, 125, 125, 62, 62, 
    62, 62, 250, 375, 125, 250, 250, 125, 125, 125,

    //DOEH DEH DEH AH DAH DOOEH DOO AH  (INTENSIFIES)
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,

    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,

    //DU DU DUDU DU DU DU
    250, 125, 125, 250, 250, 250, 625, 250, 125, 125,
    250, 250, 42, 42, 42, 125, 125, 125, 125, 250,
    125, 125, 250, 125, 250, 250, 375, 250, 250, 125,
    125, 125, 625, 500,

    //DU DU DUDU DU DU DUU  (INTENSIFIES)
    250, 125, 125, 250, 250, 250, 625, 250, 125, 125,
    250, 125, 250, 125, 125, 250, 250, 250, 250, 250,
    250, 250, 250, 250, 250, 125, 125, 250, 1125, 

    //Epic part
    125, 125, 125, 125, 125, 125, 125, 125, 63, 63,
    63, 63, 250, 1125, 250, 125, 250, 125, 125, 125,
    125, 125, 125, 250, 250, 250, 250, 250, 250, 125,
    125, 125, 1125, 250, 250, 250, 250, 500, 500, 500,
    500, 500, 500, 1000, 125, 125, 125, 125, 125, 125,
    125, 125, 1000, 1000, 125, 125, 125, 125, 125, 125,
    125, 125, 63, 63, 63, 63, 250, 1125, 250, 125,
    250, 125, 125, 125, 125, 125, 125, 250, 250, 250,
    250, 250, 250, 125, 125, 125, 1125, 250, 250, 250,
    250, 500, 500, 500, 500, 500, 500, 1000, 125, 125, 
    125, 125, 125, 125, 125, 125, 1000, 1000,

    //Opera
    1500, 500, 1000, 1000, 4000, 1500, 500, 1000, 1000, 1000,
    83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 
    83, 83, 2000, 1500, 500, 1000, 1000, 2000, 125, 125,
    125, 125, 250, 125, 125, 125, 250, 250, 125, 125,
    1500, 500, 1000, 1000, 125, 125, 250, 250, 125, 250, 
    250, 125, 125, 125, 125, 125, 250, 250, 125, 250,
    250, 125, 125, 125,

    //Rock part
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250,
    125, 125, 250, 125, 250, 250, 250, 250, 125, 125,
    125, 125, 125, 250, 125, 250, 250, 250, 250, 125,
    125, 125,

    //End part
    125, 125, 250, 250, 125, 250, 250, 125, 125, 125,
    125, 125, 125, 125, 
    250, 375, 125, 250, 250, 125, 125, 125, 125, 125, 
    250, 375, 125, 250, 250, 125, 125, 125, 62, 62,
    62, 62, 250, 375, 125, 250, 250, 125, 125, 125, 
    125, 125, 250, 375, 125, 250, 250, 125, 125, 125, 
    125, 125, 125, 125, 
    250, 375, 125, 250, 250, 125, 125, 125, 
};

uint32_t Delay_of_Megalovania[] = {// (ms)
    //DOEH DEH DEH AH DAH DOOEH DOO AH
    125, 125, 250, 375, 250, 250, 250, 125, 125, 125, 
    62, 62, 62, 62, 
    250, 375, 250, 250, 250, 125, 125, 125, 125, 125, 
    250, 375, 250, 250, 250, 125, 125, 125, 62, 62, 
    62, 62, 250, 375, 250, 250, 250, 125, 125, 125, 
    
    125, 125, 250, 375, 250, 250, 250, 125, 125, 125, 
    62, 62, 62, 62, 
    250, 375, 250, 250, 250, 125, 125, 125, 125, 125, 
    250, 375, 250, 250, 250, 125, 125, 125, 62, 62, 
    62, 62, 250, 375, 250, 250, 250, 125, 125, 125,

    //DOEH DEH DEH AH DAH DOOEH DOO AH (INTENSIFIES)
    125, 125, 250, 325, 250, 250, 250, 125, 125, 125,
    125, 125, 250, 325, 250, 250, 250, 125, 125, 125,
    125, 125, 250, 325, 250, 250, 250, 125, 125, 125, 
    125, 125, 250, 325, 250, 250, 250, 125, 125, 125,

    125, 125, 250, 325, 250, 250, 250, 125, 125, 125, 
    125, 125, 250, 325, 250, 250, 250, 125, 125, 125, 
    125, 125, 250, 325, 250, 250, 250, 125, 125, 125, 
    125, 125, 250, 325, 250, 250, 250, 125, 125, 125,

    //DU DU DUDU DU DU DU
    250, 125, 250, 250, 250, 250, 625, 250, 125, 250, 
    250, 250, 42, 42, 42, 125, 125, 125, 375, 250, 
    125, 250, 250, 250, 250, 250, 375, 250, 250, 125, 
    125, 125, 625, 500, 

    //DU DU DUDU DU DU DUU (INTENSIFIES)
    250, 125, 250, 250, 250, 250, 625, 250, 125, 250, 
    250, 250, 250, 250, 125, 250, 250, 250, 250, 250, 
    250, 250, 250, 250, 250, 125, 250, 250, 2125, 

    //Epic part
    125, 125, 125, 125, 125, 125, 125, 125, 63, 63, 
    63, 63, 250, 1125, 250, 125, 250, 125, 125, 125, 
    125, 125, 125, 250, 250, 250, 250, 250, 250, 125, 
    125, 125, 1125, 250, 250, 250, 250, 500, 500, 500, 
    500, 500, 500, 1000, 125, 125, 125, 125, 125, 125, 
    125, 125, 1000, 2000, 125, 125, 125, 125, 125, 125, 
    125, 125, 63, 63, 63, 63, 250, 1125, 250, 125, 
    250, 125, 125, 125, 125, 125, 125, 250, 250, 250, 
    250, 250, 250, 125, 125, 125, 1125, 250, 250, 250, 
    250, 500, 500, 500, 500, 500, 500, 1000, 125, 125, 
    125, 125, 125, 125, 125, 125, 1000, 1000, 

    //Opera
    1500, 500, 1000, 1000, 4000, 1500, 500, 1000, 1000, 1000, 
    83, 83, 83, 83, 83, 83, 83, 83, 83, 83, 
    83, 83, 2000, 1500, 500, 1000, 1000, 2000, 125, 125, 
    125, 125, 250, 125, 125, 125, 250, 250, 125, 125, 
    1500, 500, 1000, 1000, 125, 125, 250, 250, 250, 250, 
    250, 125, 125, 125, 125, 125, 250, 250, 250, 250, 
    250, 125, 125, 125, 

    //Rock part
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    250, 250, 125, 250, 250, 250, 125, 125, 125, 250, 
    125, 125, 250, 125, 250, 250, 250, 250, 125, 125, 
    125, 125, 125, 250, 125, 250, 250, 250, 250, 125, 
    125, 125, 

    //End part
    125, 125, 250, 375, 250, 250, 250, 125, 125, 125, 
    62, 62, 62, 62, 
    250, 375, 250, 250, 250, 125, 125, 125, 125, 125, 
    250, 375, 250, 250, 250, 125, 125, 125, 62, 62, 
    62, 62, 250, 375, 250, 250, 250, 125, 125, 125, 
    125, 125, 250, 375, 250, 250, 250, 125, 125, 125, 
    62, 62, 62, 62, 
    250, 375, 250, 250, 250, 125, 125, 4125, 
};

uint32_t Melody_of_Bloody_Stream_JojoBA[] = {
    //Intro
    C4, Eb4, G4, Gb4, F4, F4, C4, Eb4, F4, C4, 
    Eb4, G4, Gb4, F4, F4, C4, Eb4, F4, C4, Eb4, 
    G4, Gb4, F4, F4, C4, Eb4, Bb4,

    //First part first verse
    G4, G4, Gb4, G4, Eb4, F4, Gb4, G4, Eb4, F4,
    C5, G4, F4, Eb4, F4, F4, C4, Eb4, C4, Eb4, 
    Eb4, F4, Eb4, Gb4, G4, Eb5, F5, C5, Eb5, Bb4, 
    C5, G4, C4, Eb4, Bb4, G4, Eb5, C4,

    //First part second part
    Gb4, G4, Eb4, F4, Gb4, G4, Eb4, F4, Ab4, Eb5, 
    Db5, C5, C5, C5, D5, Eb5, D5, C5, D5, C5, 
    Eb4, Eb4, Eb4, Bb4, Ab4, G4, G5, Ab5, Bb5, Bb5, 
    Eb5, Eb5, F5, F5, G5, G5, B4, C5, D5, G4, 
    A4, B4, G4, F4, Eb4, D4, 
    
    //Second part
    C5, C5, Bb4, C5, C5, D5, Eb5, D5, Bb4, Bb4, 
    F4, G4, F4, G4, G4, C4, Eb4, G4, Ab4, Bb4, 
    G4, B3, C4, D4, Ab3, Bb3, C4, D4, Eb4, F4, 
    G4, Ab4, 

    //Third part first verse
    G4, B4, D5, Eb5, D5, C5, Bb4, C5, G4, C5, 
    Eb5, G5, F5, Eb5, F5, Eb5, Bb4, Bb4, C5, C5, 
    C5, C5, C5, C5, C5, C5, C5, C5, B4, B4, 
    C5, D5, Eb5, D5, C5, Bb4, C5, G4, C5, Eb5, 
    G5, Bb4, Ab5, G5, F5, G5, F5, Eb5, G4, Ab4, 
    Bb4, D5, Eb5, F5, Ab5, Bb5, B5, G4,

    //Third part second verse
    Eb5, D5, C5, Bb4, C5, G4, C5, Eb5, G5, F5, 
    Eb5, F5, Eb5, Bb4, Bb4, C5, C5, C5, C5, C5, 
    C5, C5, C5, C5, C5, B4, B4, C5, D5, Eb5, 
    D5, C5, Bb4, C5, G4, C5, Eb5, G5, F5, Eb5, 
    F5, Eb5, G4, G4, G5, 

    //ending
    G4, D5, F5, Eb5, D5, C5, C4
};

uint32_t duration_of_Bloody_Stream_JojoBA[] = {
    //Intro
    114, 114, 227, 341, 227, 227, 114, 227, 114, 114, 
    114, 227, 341, 227, 227, 114, 227, 114, 114, 114, 
    227, 341, 227, 227, 114, 227, 910,

    //First part first verse
    455, 455, 682, 1137, 455, 455, 682, 682, 455, 1365, 
    227, 227, 227, 114, 227, 227, 114, 227, 114, 1231, 
    227, 227, 227, 682, 1137, 114, 227, 114, 227, 114, 
    227, 114, 114, 114, 114, 114, 114, 114, 

    //First part second part
    682, 1137, 455, 455, 682, 682, 455, 455, 455, 455, 
    455, 227, 227, 227, 114, 341, 227, 227, 114, 910, 
    227, 227, 227, 227, 114, 910, 114, 114, 104, 10, 
    104, 10, 10, 104, 10, 104, 114, 114, 114, 114, 
    114, 114, 114, 114, 114, 114, 

    //Second part
    455, 227, 114, 910, 341, 341, 227, 1137, 227, 227, 
    114, 1024, 455, 227, 682, 227, 227, 341, 341, 227, 
    455, 455, 455, 455, 114, 227, 227, 227, 227, 227, 
    227, 227, 

    //Third part first verse
    455, 455, 455, 227, 227, 114, 227, 114, 227, 227, 
    227, 227, 227, 114, 227, 114, 114, 114, 227, 227, 
    227, 114, 227, 227, 114, 227, 227, 227, 227, 114, 
    227, 1024, 227, 227, 114, 227, 114, 227, 227, 227, 
    682, 227, 227, 227, 227, 227, 682, 1137, 114, 114, 
    114, 114, 114, 114, 114, 114, 227, 682, 
 
    //Third part second verse
    227, 227, 114, 227, 114, 227, 227, 227, 227, 227, 
    114, 227, 114, 114, 114, 227, 227, 227, 114, 227, 
    227, 114, 227, 227, 227, 227, 114, 227, 1024, 227, 
    227, 114, 227, 114, 227, 227, 227, 227, 227, 114, 
    227, 114, 227, 227, 1592, 
    
    //ending
    910, 455, 227, 144, 227, 3526, 3185
};

uint32_t delay_of_Bloody_Stream_JojoBA[] = {
    //Intro
    114, 114, 227, 341, 227, 227, 114, 227, 227, 114, 
    114, 227, 341, 227, 227, 114, 227, 227, 114, 114, 
    227, 341, 227, 227, 114, 227, 1137, 

    //First part first verse
    455, 455, 682, 2047, 455, 455, 682, 682, 455, 1592, 
    227, 227, 227, 114, 227, 227, 114, 227, 114, 1231, 
    227, 227, 227, 682, 1137, 114, 227, 114, 227, 114, 
    227, 114, 114, 114, 114, 114, 114, 114,  

    //First part second part
    682, 2047, 455, 455, 682, 682, 455, 455, 455, 455, 
    455, 227, 227, 227, 114, 341, 227, 227, 114, 1365, 
    227, 227, 227, 227, 114, 1479, 114, 114, 104, 10, 
    104, 10, 10, 104, 10, 104, 114, 114, 114, 114, 
    114, 114, 114, 114, 114, 114,  
    
    //Second part
    455, 227, 114, 1934, 341, 341, 227, 1137, 227, 227, 
    114, 1934, 455, 227, 682, 227, 1137, 341, 341, 227, 
    455, 455, 455, 455, 114, 227, 227, 227, 227, 227, 
    227, 796, 

    //Third part first verse
    455, 455, 455, 227, 227, 114, 227, 227, 227, 227, 
    227, 227, 227, 114, 227, 796, 114, 114, 227, 227, 
    227, 114, 227, 227, 114, 227, 227, 227, 227, 114, 
    227, 1024, 227, 227, 114, 227, 227, 227, 227, 227, 
    682, 227, 227, 227, 227, 227, 682, 1137, 114, 114, 
    114, 114, 114, 114, 114, 114, 227, 682,  
    
    //Third part second verse
    227, 227, 114, 227, 227, 227, 227, 227, 227, 227, 
    114, 227, 796, 114, 114, 227, 227, 227, 114, 227, 
    227, 114, 227, 227, 227, 227, 114, 227, 1024, 227, 
    227, 114, 227, 227, 227, 227, 227, 227, 227, 114, 
    227, 341, 227, 227, 1820,  

    //ending
    910, 455, 227, 144, 227, 3526, 10000
};
uint32_t Pirates_note[] = {
	D4, D4, D4, D4, D4, D4, D4, D4,
	D4, D4, D4, D4, D4, D4, D4, D4,
	D4, D4, D4, D4, D4, D4, D4, D4,
	A3, C4, D4, D4, D4, E4, F4, F4,
	F4, G4, E4, E4, D4, C4, C4, D4,
	0,  A3, C4, B3, D4, B3, E4, F4,
	F4, C4, C4, C4, C4, D4, C4,
	D4, 0,  0,  A3, C4, D4, D4, D4, F4,
	G4, G4, G4, A4, A4, A4, A4, G4,
	A4, D4, 0,  D4, E3, F4, F4, G4, A4,
	D4, 0,  D4, F4, E4, E4, F4, D4
};
uint32_t Pirates_duration[] = {
	4,8,4,8,4,8,8,8,8,4,8,4,8,4,8,8,8,8,4,8,4,8,
	4,8,8,8,8,4,4,8,8,4,4,8,8,4,4,8,8,
	8,4,8,8,8,4,4,8,8,4,4,8,8,4,4,8,4,
	4,8,8,8,8,4,4,8,8,4,4,8,8,4,4,8,8,
	8,4,8,8,8,4,4,4,8,4,8,8,8,4,4,8,8
};

uint32_t Epic_Sax_Guy[] = {
	A4, A4, A4, A4, G4, A4,
	A4, A4, A4, A4, G4, A4, A4,
	C5, A4, G4, F4, D4, D4, E4, F4, D4,
	A4, A4, A4, A4, G4, A4,
	A4, A4, A4, A4, G4, A4, A4,
	C5, A4, G4, F4, D4, D4, E4, F4, D4,

};
uint32_t beat_Epic_Sax_Guy[] = {
	750, 250, 125, 125, 250, 250,
	750, 250, 125, 125, 250, 250,
	250, 250, 250, 250, 125, 125, 125, 125, 125,
	
	1000, 250, 125, 125, 250, 250,
	1000, 250, 125, 125, 250, 250,
	250, 250, 250, 250, 125, 125, 125, 125, 125,
	
};
#endif