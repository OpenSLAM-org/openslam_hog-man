// HOG-Man - Hierarchical Optimization for Pose Graphs on Manifolds
// Copyright (C) 2010 G. Grisetti, R. Kümmerle, C. Stachniss
// 
// HOG-Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// HOG-Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef POSE_GRAPH_3D_GRADIENT_H
#define POSE_GRAPH_3D_GRADIENT_H

#include "aislib/math/transformation.h"
#include <cmath>

namespace AISNavigation {

inline void manifoldGradientXi(Matrix6& mat, const Vector6& e, const Vector6& xi, const Vector6& xj)
{
  //const double& ex  = e[0];
  //const double& ey  = e[1];
  //const double& ez  = e[2];
  const double& er  = e[3];
  const double& ep  = e[4];
  const double& eya = e[5];

  const double& x1 = xi[0];
  const double& y1 = xi[1];
  const double& z1 = xi[2];
  const double& a1 = xi[3];
  const double& b1 = xi[4];
  const double& c1 = xi[5];

  const double& x2 = xj[0];
  const double& y2 = xj[1];
  const double& z2 = xj[2];
  const double& a2 = xj[3];
  const double& b2 = xj[4];
  const double& c2 = xj[5];

  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 , aux_12 ,
         aux_13 , aux_14 , aux_15 , aux_16 , aux_17 , aux_18 , aux_19 , aux_20 , aux_21 , aux_22 , aux_23 , aux_24 ,
         aux_25 , aux_26 , aux_27 , aux_28 , aux_29 , aux_30 , aux_31 , aux_32 , aux_33 , aux_34 , aux_35 , aux_36 ,
         aux_37 , aux_38 , aux_39 , aux_40 , aux_41 , aux_42 , aux_43 , aux_44 , aux_45 , aux_46 , aux_47 , aux_48 ,
         aux_49 , aux_50 , aux_51 , aux_52 , aux_53 , aux_54 , aux_55 , aux_56 , aux_57 , aux_58 , aux_59 , aux_60 ,
         aux_61 , aux_62 , aux_63 , aux_64 ;
  aux_1 = cos(a1) ;
  aux_2 = sin(b1) ;
  aux_3 = cos(c1) ;
  aux_4 = sin(a1) ;
  aux_5 = sin(c1) ;
  aux_6 = -14745600*aux_4*aux_5-14745600*aux_1*aux_2*aux_3 ;
  aux_7 = sin(ep) ;
  aux_8 = cos(b1) ;
  aux_9 = cos(ep) ;
  aux_10 = cos(eya) ;
  aux_11 = 14745600*aux_4*aux_2*aux_3-14745600*aux_1*aux_5 ;
  aux_12 = sin(eya) ;
  aux_13 = -14745600*aux_4*aux_2*aux_5-14745600*aux_1*aux_3 ;
  aux_14 = -aux_4*aux_5-aux_1*aux_2*aux_3 ;
  aux_15 = aux_4*aux_5+aux_1*aux_2*aux_3 ;
  aux_16 = aux_1*aux_2*aux_5-aux_4*aux_3 ;
  aux_17 = aux_1*aux_8*z2-aux_1*aux_8*z1+aux_16*y2-aux_16*y1+aux_15*x2+aux_14*x1 ;
  aux_18 = aux_4*aux_2*aux_3-aux_1*aux_5 ;
  aux_19 = aux_1*aux_5-aux_4*aux_2*aux_3 ;
  aux_20 = -aux_4*aux_2*aux_5-aux_1*aux_3 ;
  aux_21 = -aux_4*aux_8*z2+aux_4*aux_8*z1+aux_20*y2-aux_20*y1+aux_19*x2+aux_18*x1 ;
  aux_22 = aux_4*aux_3-aux_1*aux_2*aux_5 ;
  aux_23 = -aux_1*aux_8*z2+aux_1*aux_8*z1+aux_22*y2-aux_22*y1+aux_14*x2+aux_15*x1 ;
  aux_24 = -aux_2*z2+aux_2*z1+aux_8*aux_5*y2-aux_8*aux_5*y1+aux_8*aux_3*x2-aux_8*aux_3*x1 ;
  aux_25 = aux_4*aux_2*aux_5+aux_1*aux_3 ;
  aux_26 = aux_4*aux_8*z2-aux_4*aux_8*z1+aux_25*y2-aux_25*y1+aux_18*x2+aux_19*x1 ;
  aux_27 = aux_2*z2-aux_2*z1-aux_8*aux_5*y2+aux_8*aux_5*y1-aux_8*aux_3*x2+aux_8*aux_3*x1 ;
  aux_28 = sin(er) ;
  aux_29 = cos(er) ;
  aux_30 = 14745600*aux_4*aux_3-14745600*aux_1*aux_2*aux_5 ;
  aux_31 = aux_7*aux_28*aux_12+aux_29*aux_10 ;
  aux_32 = aux_7*aux_28*aux_10-aux_29*aux_12 ;
  aux_33 = aux_7*aux_29*aux_12-aux_28*aux_10 ;
  aux_34 = aux_28*aux_12+aux_7*aux_29*aux_10 ;
  aux_35 = 0 ;
  aux_36 = sin(a2) ;
  aux_37 = cos(b2) ;
  aux_38 = sin(b2) ;
  aux_39 = cos(c2) ;
  aux_40 = cos(a2) ;
  aux_41 = sin(c2) ;
  aux_42 = aux_36*aux_38*aux_39-aux_40*aux_41 ;
  aux_43 = aux_36*aux_38*aux_41+aux_40*aux_39 ;
  aux_44 = aux_16*aux_43+aux_15*aux_42+aux_1*aux_36*aux_8*aux_37 ;
  aux_45 = aux_36*aux_41+aux_40*aux_38*aux_39 ;
  aux_46 = aux_40*aux_38*aux_41-aux_36*aux_39 ;
  aux_47 = aux_16*aux_46+aux_15*aux_45+aux_1*aux_40*aux_8*aux_37 ;
  aux_48 = aux_25*aux_46+aux_18*aux_45+aux_4*aux_40*aux_8*aux_37 ;
  aux_49 = aux_8*aux_5*aux_46+aux_8*aux_3*aux_45-aux_40*aux_2*aux_37 ;
  aux_50 = aux_49*aux_34+aux_48*aux_33+aux_47*aux_9*aux_29 ;
  aux_51 = aux_25*aux_43+aux_18*aux_42+aux_4*aux_36*aux_8*aux_37 ;
  aux_52 = aux_8*aux_5*aux_43+aux_8*aux_3*aux_42-aux_36*aux_2*aux_37 ;
  aux_53 = aux_52*aux_34+aux_51*aux_33+aux_44*aux_9*aux_29 ;
  aux_54 = 1/(pow(aux_53,2)+pow(aux_50,2)) ;
  aux_55 = aux_37*aux_20*aux_41+aux_37*aux_19*aux_39+aux_4*aux_8*aux_38 ;
  aux_56 = aux_37*aux_16*aux_41+aux_37*aux_15*aux_39-aux_1*aux_8*aux_38 ;
  aux_57 = aux_37*aux_25*aux_41+aux_37*aux_18*aux_39-aux_4*aux_8*aux_38 ;
  aux_58 = aux_8*aux_37*aux_5*aux_41+aux_8*aux_37*aux_3*aux_39+aux_2*aux_38 ;
  aux_59 = 1/sqrt(1-pow(aux_58*aux_34+aux_57*aux_33+aux_56*aux_9*aux_29,2)) ;
  aux_60 = aux_37*aux_22*aux_41+aux_37*aux_14*aux_39+aux_1*aux_8*aux_38 ;
  aux_61 = -aux_8*aux_37*aux_5*aux_41-aux_8*aux_37*aux_3*aux_39-aux_2*aux_38 ;
  aux_62 = aux_57*aux_9*aux_12+aux_58*aux_9*aux_10-aux_56*aux_7 ;
  aux_63 = aux_57*aux_31+aux_58*aux_32+aux_56*aux_9*aux_28 ;
  aux_64 = 1/(pow(aux_63,2)+pow(aux_62,2)) ;


  mat[0][0] = -(aux_11*aux_9*aux_12+14745600*aux_8*aux_3*aux_9*aux_10+aux_6*aux_7)/14745600;
  mat[0][1] = (aux_13*aux_9*aux_12-14745600*aux_8*aux_5*aux_9*aux_10+(14745600*aux_1*aux_2*aux_5-14745600*aux_4*aux_3)*aux_7)/14745600;
  mat[0][2] = (-14745600*aux_4*aux_8*aux_9*aux_12+14745600*aux_2*aux_9*aux_10+14745600*aux_1*aux_8*aux_7)/14745600;
  mat[0][3] = aux_9*aux_12*aux_17-aux_7*aux_21;
  mat[0][4] = aux_9*aux_10*aux_23-aux_7*aux_24;
  mat[0][5] = aux_9*aux_12*aux_27+aux_9*aux_10*aux_26;
  mat[1][0] = -((aux_11*aux_7*aux_28-14745600*aux_8*aux_3*aux_29)*aux_12+(14745600*aux_8*aux_3*aux_7*aux_28+aux_11*aux_29)*aux_10+(14745600*aux_4*aux_5+14745600*aux_1*aux_2*aux_3)*aux_9*aux_28)/14745600;
  mat[1][1] = ((aux_13*aux_7*aux_28+14745600*aux_8*aux_5*aux_29)*aux_12+(aux_13*aux_29-14745600*aux_8*aux_5*aux_7*aux_28)*aux_10+aux_30*aux_9*aux_28)/14745600;
  mat[1][2] = ((-14745600*aux_4*aux_8*aux_7*aux_28-14745600*aux_2*aux_29)*aux_12+(14745600*aux_2*aux_7*aux_28-14745600*aux_4*aux_8*aux_29)*aux_10-14745600*aux_1*aux_8*aux_9*aux_28)/14745600;
  mat[1][3] = aux_9*aux_28*aux_21+aux_31*aux_17;
  mat[1][4] = aux_9*aux_28*aux_24+aux_32*aux_23;
  mat[1][5] = aux_31*aux_27+aux_32*aux_26;
  mat[2][0] = (((14745600*aux_1*aux_5-14745600*aux_4*aux_2*aux_3)*aux_7*aux_29-14745600*aux_8*aux_3*aux_28)*aux_12+(aux_11*aux_28-14745600*aux_8*aux_3*aux_7*aux_29)*aux_10+aux_6*aux_9*aux_29)/14745600;
  mat[2][1] = ((aux_13*aux_7*aux_29-14745600*aux_8*aux_5*aux_28)*aux_12+((14745600*aux_4*aux_2*aux_5+14745600*aux_1*aux_3)*aux_28-14745600*aux_8*aux_5*aux_7*aux_29)*aux_10+aux_30*aux_9*aux_29)/14745600;
  mat[2][2] = -((14745600*aux_4*aux_8*aux_7*aux_29-14745600*aux_2*aux_28)*aux_12+(-14745600*aux_4*aux_8*aux_28-14745600*aux_2*aux_7*aux_29)*aux_10+14745600*aux_1*aux_8*aux_9*aux_29)/14745600;
  mat[2][3] = aux_9*aux_29*aux_21+aux_33*aux_17;
  mat[2][4] = aux_9*aux_29*aux_24+aux_34*aux_23;
  mat[2][5] = aux_33*aux_27+aux_34*aux_26;
  mat[3][0] = aux_35;
  mat[3][1] = aux_35;
  mat[3][2] = aux_35;
  mat[3][3] = (aux_44*aux_33+(aux_20*aux_43+aux_19*aux_42-aux_4*aux_36*aux_8*aux_37)*aux_9*aux_29)*aux_50*aux_54-(aux_47*aux_33+(aux_20*aux_46+aux_19*aux_45-aux_4*aux_40*aux_8*aux_37)*aux_9*aux_29)*aux_53*aux_54;
  mat[3][4] = aux_50*((aux_22*aux_43+aux_14*aux_42-aux_1*aux_36*aux_8*aux_37)*aux_34+aux_52*aux_9*aux_29)*aux_54-((aux_22*aux_46+aux_14*aux_45-aux_1*aux_40*aux_8*aux_37)*aux_34+aux_49*aux_9*aux_29)*aux_53*aux_54;
  mat[3][5] = aux_50*(aux_51*aux_34+(-aux_8*aux_5*aux_43-aux_8*aux_3*aux_42+aux_36*aux_2*aux_37)*aux_33)*aux_54-(aux_48*aux_34+(-aux_8*aux_5*aux_46-aux_8*aux_3*aux_45+aux_40*aux_2*aux_37)*aux_33)*aux_53*aux_54;
  mat[4][0] = aux_35;
  mat[4][1] = aux_35;
  mat[4][2] = aux_35;
  mat[4][3] = -(aux_56*aux_33+aux_55*aux_9*aux_29)*aux_59;
  mat[4][4] = -(aux_60*aux_34+aux_58*aux_9*aux_29)*aux_59;
  mat[4][5] = -(aux_57*aux_34+aux_61*aux_33)*aux_59;
  mat[5][0] = aux_35;
  mat[5][1] = aux_35;
  mat[5][2] = aux_35;
  mat[5][3] = aux_62*(aux_56*aux_31+aux_55*aux_9*aux_28)*aux_64-(aux_56*aux_9*aux_12-aux_55*aux_7)*aux_63*aux_64;
  mat[5][4] = aux_62*(aux_60*aux_32+aux_58*aux_9*aux_28)*aux_64-(aux_60*aux_9*aux_10-aux_58*aux_7)*aux_63*aux_64;
  mat[5][5] = aux_62*(aux_61*aux_31+aux_57*aux_32)*aux_64-(aux_61*aux_9*aux_12+aux_57*aux_9*aux_10)*aux_63*aux_64;
}  

inline void manifoldGradientXj(Matrix6& mat, const Vector6& e, const Vector6& xi, const Vector6& xj)
{
  //const double& ex  = e[0];
  //const double& ey  = e[1];
  //const double& ez  = e[2];
  const double& er  = e[3];
  const double& ep  = e[4];
  const double& eya = e[5];

  //const double& x1 = xi[0];
  //const double& y1 = xi[1];
  //const double& z1 = xi[2];
  const double& a1 = xi[3];
  const double& b1 = xi[4];
  const double& c1 = xi[5];

  //const double& x2 = xj[0];
  //const double& y2 = xj[1];
  //const double& z2 = xj[2];
  const double& a2 = xj[3];
  const double& b2 = xj[4];
  const double& c2 = xj[5];

  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 , aux_12 , aux_13 ,
         aux_14 , aux_15 , aux_16 , aux_17 , aux_18 , aux_19 , aux_20 , aux_21 , aux_22 , aux_23 , aux_24 , aux_25 ,
         aux_26 , aux_27 , aux_28 , aux_29 , aux_30 , aux_31 , aux_32 , aux_33 , aux_34 , aux_35 , aux_36 , aux_37 ,
         aux_38 , aux_39 , aux_40 , aux_41 , aux_42 , aux_43 , aux_44 , aux_45 , aux_46 , aux_47 , aux_48 , aux_49 ,
         aux_50 , aux_51 , aux_52 , aux_53 ;
  aux_1 = cos(a1) ;
  aux_2 = sin(b1) ;
  aux_3 = cos(c1) ;
  aux_4 = sin(a1) ;
  aux_5 = sin(c1) ;
  aux_6 = sin(ep) ;
  aux_7 = cos(b1) ;
  aux_8 = cos(ep) ;
  aux_9 = cos(eya) ;
  aux_10 = aux_4*aux_2*aux_3-aux_1*aux_5 ;
  aux_11 = sin(eya) ;
  aux_12 = aux_4*aux_2*aux_5+aux_1*aux_3 ;
  aux_13 = 0 ;
  aux_14 = aux_4*aux_5+aux_1*aux_2*aux_3 ;
  aux_15 = sin(er) ;
  aux_16 = cos(er) ;
  aux_17 = aux_1*aux_2*aux_5-aux_4*aux_3 ;
  aux_18 = cos(a2) ;
  aux_19 = cos(b2) ;
  aux_20 = sin(b2) ;
  aux_21 = cos(c2) ;
  aux_22 = sin(a2) ;
  aux_23 = sin(c2) ;
  aux_24 = aux_22*aux_23+aux_18*aux_20*aux_21 ;
  aux_25 = aux_18*aux_20*aux_23-aux_22*aux_21 ;
  aux_26 = aux_6*aux_16*aux_11-aux_15*aux_9 ;
  aux_27 = aux_15*aux_11+aux_6*aux_16*aux_9 ;
  aux_28 = (aux_7*aux_5*aux_25+aux_7*aux_3*aux_24-aux_18*aux_2*aux_19)*aux_27+(aux_12*aux_25+aux_10*aux_24+aux_4*aux_18*aux_7*aux_19)*aux_26+(aux_17*aux_25+aux_14*aux_24+aux_1*aux_18*aux_7*aux_19)*aux_8*aux_16 ;
  aux_29 = pow(aux_28,2) ;
  aux_30 = aux_22*aux_20*aux_21-aux_18*aux_23 ;
  aux_31 = aux_22*aux_20*aux_23+aux_18*aux_21 ;
  aux_32 = aux_17*aux_31+aux_14*aux_30+aux_1*aux_22*aux_7*aux_19 ;
  aux_33 = aux_12*aux_31+aux_10*aux_30+aux_4*aux_22*aux_7*aux_19 ;
  aux_34 = aux_7*aux_5*aux_31+aux_7*aux_3*aux_30-aux_22*aux_2*aux_19 ;
  aux_35 = aux_34*aux_27+aux_33*aux_26+aux_32*aux_8*aux_16 ;
  aux_36 = 1/(pow(aux_35,2)+aux_29) ;
  aux_37 = aux_18*aux_23-aux_22*aux_20*aux_21 ;
  aux_38 = -aux_22*aux_20*aux_23-aux_18*aux_21 ;
  aux_39 = aux_19*aux_17*aux_23+aux_19*aux_14*aux_21-aux_1*aux_7*aux_20 ;
  aux_40 = aux_19*aux_12*aux_23+aux_19*aux_10*aux_21-aux_4*aux_7*aux_20 ;
  aux_41 = aux_7*aux_19*aux_5*aux_23+aux_7*aux_19*aux_3*aux_21+aux_2*aux_20 ;
  aux_42 = aux_41*aux_27+aux_40*aux_26+aux_39*aux_8*aux_16 ;
  aux_43 = -aux_22*aux_23-aux_18*aux_20*aux_21 ;
  aux_44 = aux_22*aux_21-aux_18*aux_20*aux_23 ;
  aux_45 = aux_17*aux_44+aux_14*aux_43-aux_1*aux_18*aux_7*aux_19 ;
  aux_46 = aux_12*aux_44+aux_10*aux_43-aux_4*aux_18*aux_7*aux_19 ;
  aux_47 = aux_7*aux_5*aux_44+aux_7*aux_3*aux_43+aux_18*aux_2*aux_19 ;
  aux_48 = 1/sqrt(1-pow(aux_42,2)) ;
  aux_49 = aux_6*aux_15*aux_9-aux_16*aux_11 ;
  aux_50 = aux_6*aux_15*aux_11+aux_16*aux_9 ;
  aux_51 = aux_40*aux_50+aux_41*aux_49+aux_39*aux_8*aux_15 ;
  aux_52 = aux_40*aux_8*aux_11+aux_41*aux_8*aux_9-aux_39*aux_6 ;
  aux_53 = 1/(pow(aux_52,2)+pow(aux_51,2)) ;

  mat[0][0] = aux_10*aux_8*aux_11+aux_7*aux_3*aux_8*aux_9+(-aux_4*aux_5-aux_1*aux_2*aux_3)*aux_6;
  mat[0][1] = aux_12*aux_8*aux_11+aux_7*aux_5*aux_8*aux_9+(aux_4*aux_3-aux_1*aux_2*aux_5)*aux_6;
  mat[0][2] = aux_4*aux_7*aux_8*aux_11-aux_2*aux_8*aux_9-aux_1*aux_7*aux_6;
  mat[0][3] = aux_13;
  mat[0][4] = aux_13;
  mat[0][5] = aux_13;
  mat[1][0] = (aux_10*aux_6*aux_15-aux_7*aux_3*aux_16)*aux_11+(aux_7*aux_3*aux_6*aux_15+aux_10*aux_16)*aux_9+aux_14*aux_8*aux_15;
  mat[1][1] = (aux_12*aux_6*aux_15-aux_7*aux_5*aux_16)*aux_11+(aux_7*aux_5*aux_6*aux_15+aux_12*aux_16)*aux_9+aux_17*aux_8*aux_15;
  mat[1][2] = (aux_4*aux_7*aux_6*aux_15+aux_2*aux_16)*aux_11+(aux_4*aux_7*aux_16-aux_2*aux_6*aux_15)*aux_9+aux_1*aux_7*aux_8*aux_15;
  mat[1][3] = aux_13;
  mat[1][4] = aux_13;
  mat[1][5] = aux_13;
  mat[2][0] = (aux_7*aux_3*aux_15+aux_10*aux_6*aux_16)*aux_11+((aux_1*aux_5-aux_4*aux_2*aux_3)*aux_15+aux_7*aux_3*aux_6*aux_16)*aux_9+aux_14*aux_8*aux_16;
  mat[2][1] = (aux_7*aux_5*aux_15+aux_12*aux_6*aux_16)*aux_11+((-aux_4*aux_2*aux_5-aux_1*aux_3)*aux_15+aux_7*aux_5*aux_6*aux_16)*aux_9+aux_17*aux_8*aux_16;
  mat[2][2] = (aux_4*aux_7*aux_6*aux_16-aux_2*aux_15)*aux_11+(-aux_4*aux_7*aux_15-aux_2*aux_6*aux_16)*aux_9+aux_1*aux_7*aux_8*aux_16;
  mat[2][3] = aux_13;
  mat[2][4] = aux_13;
  mat[2][5] = aux_13;
  mat[3][0] = aux_13;
  mat[3][1] = aux_13;
  mat[3][2] = aux_13;
  mat[3][3] = aux_29*aux_36-((aux_7*aux_5*aux_38+aux_7*aux_3*aux_37+aux_22*aux_2*aux_19)*aux_27+(aux_12*aux_38+aux_10*aux_37-aux_4*aux_22*aux_7*aux_19)*aux_26+(aux_17*aux_38+aux_14*aux_37-aux_1*aux_22*aux_7*aux_19)*aux_8*aux_16)*aux_35*aux_36;
  mat[3][4] = -aux_42*aux_35*aux_36;
  mat[3][5] = ((-aux_7*aux_19*aux_5*aux_23-aux_7*aux_19*aux_3*aux_21-aux_2*aux_20)*aux_27+(-aux_19*aux_12*aux_23-aux_19*aux_10*aux_21+aux_4*aux_7*aux_20)*aux_26+(-aux_19*aux_17*aux_23-aux_19*aux_14*aux_21+aux_1*aux_7*aux_20)*aux_8*aux_16)*aux_28*aux_36;
  mat[4][0] = aux_13;
  mat[4][1] = aux_13;
  mat[4][2] = aux_13;
  mat[4][3] = aux_13;
  mat[4][4] = -(aux_47*aux_27+aux_46*aux_26+aux_45*aux_8*aux_16)*aux_48;
  mat[4][5] = -aux_35*aux_48;
  mat[5][0] = aux_13;
  mat[5][1] = aux_13;
  mat[5][2] = aux_13;
  mat[5][3] = aux_13;
  mat[5][4] = aux_52*(aux_46*aux_50+aux_47*aux_49+aux_45*aux_8*aux_15)*aux_53-(aux_46*aux_8*aux_11+aux_47*aux_8*aux_9-aux_45*aux_6)*aux_51*aux_53;
  mat[5][5] = aux_52*(aux_33*aux_50+aux_34*aux_49+aux_32*aux_8*aux_15)*aux_53-(aux_33*aux_8*aux_11+aux_34*aux_8*aux_9-aux_32*aux_6)*aux_51*aux_53;
}  

inline void manifold2euler(Matrix6& mat, double manifold[6])
{
  //const double& mx = manifold[0];
  //const double& my = manifold[1];
  //const double& mz = manifold[2];
  const double& ma = manifold[3];
  const double& mb = manifold[4];
  const double& mc = manifold[5];

  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 , aux_12 ,
         aux_13 , aux_14 , aux_15 , aux_16 , aux_17 , aux_18 , aux_19 , aux_20 , aux_21 , aux_22 , aux_23 ,
         aux_24 , aux_25 , aux_26 , aux_27 , aux_28 , aux_29 , aux_30 , aux_31 , aux_32 , aux_33 , aux_34 ,
         aux_35 , aux_36 , aux_37 , aux_38 , aux_39 , aux_40 , aux_41 , aux_42 , aux_43 , aux_44 , aux_45 ,
         aux_46 , aux_47 , aux_48 , aux_49 , aux_50 , aux_51 , aux_52 , aux_53 , aux_54 ;
  aux_1 = 1 ;
  aux_2 = 0 ;
  aux_3 = std::pow(ma,3) ;
  aux_4 = std::pow(ma,2) ;
  aux_5 = std::pow(mb,2) ;
  aux_6 = std::pow(mc,2) ;
  aux_7 = aux_6+aux_5+aux_4 ;
  aux_8 = sqrt(aux_7) ;
  aux_9 = 1/std::pow(aux_8,3) ;
  aux_10 = aux_8/2 ;
  aux_11 = cos(aux_10) ;
  aux_12 = sin(aux_10) ;
  aux_13 = -ma*aux_5*aux_9*aux_11*aux_12 ;
  aux_14 = ma*aux_6*aux_9*aux_11*aux_12 ;
  aux_15 = 1/aux_8 ;
  aux_16 = -ma*aux_15*aux_11*aux_12 ;
  aux_17 = 1/std::pow(aux_7,2) ;
  aux_18 = std::pow(aux_12,2) ;
  aux_19 = 2*ma*aux_5*aux_17*aux_18 ;
  aux_20 = -2*ma*aux_6*aux_17*aux_18 ;
  aux_21 = 1/aux_7 ;
  aux_22 = mb*mc*aux_21*aux_18+ma*aux_15*aux_11*aux_12 ;
  aux_23 = std::pow(aux_11,2) ;
  aux_24 = -aux_5*aux_21*aux_18 ;
  aux_25 = aux_6*aux_21*aux_18+aux_24-aux_4*aux_21*aux_18+aux_23 ;
  aux_26 = 1/(std::pow(aux_25,2)+4*std::pow(aux_22,2)) ;
  aux_27 = ma*mb*mc*aux_9*aux_11*aux_12 ;
  aux_28 = aux_15*aux_11*aux_12 ;
  aux_29 = -2*ma*mb*mc*aux_17*aux_18 ;
  aux_30 = std::pow(mb,3) ;
  aux_31 = -aux_30*aux_9*aux_11*aux_12 ;
  aux_32 = mb*aux_6*aux_9*aux_11*aux_12 ;
  aux_33 = -mb*aux_15*aux_11*aux_12 ;
  aux_34 = 2*aux_30*aux_17*aux_18 ;
  aux_35 = -2*mb*aux_6*aux_17*aux_18 ;
  aux_36 = -2*mb*aux_21*aux_18 ;
  aux_37 = mc*aux_21*aux_18 ;
  aux_38 = -aux_5*mc*aux_9*aux_11*aux_12 ;
  aux_39 = std::pow(mc,3) ;
  aux_40 = -mc*aux_15*aux_11*aux_12 ;
  aux_41 = 2*aux_5*mc*aux_17*aux_18 ;
  aux_42 = ma*mc*aux_21*aux_23/2 ;
  aux_43 = -ma*mc*aux_9*aux_11*aux_12 ;
  aux_44 = mb*aux_21*aux_18 ;
  aux_45 = -ma*mc*aux_21*aux_18/2 ;
  aux_46 = aux_4*mc*aux_9*aux_11*aux_12 ;
  aux_47 = -2*aux_4*mc*aux_17*aux_18 ;
  aux_48 = 1/sqrt(1-4*std::pow(ma*mc*aux_21*aux_18+aux_33,2)) ;
  aux_49 = ma*aux_21*aux_18 ;
  aux_50 = ma*mb*aux_21*aux_18+mc*aux_15*aux_11*aux_12 ;
  aux_51 = -aux_6*aux_21*aux_18+aux_24+aux_4*aux_21*aux_18+aux_23 ;
  aux_52 = 1/(std::pow(aux_51,2)+4*std::pow(aux_50,2)) ;
  aux_53 = aux_4*mb*aux_9*aux_11*aux_12 ;
  aux_54 = -2*aux_4*mb*aux_17*aux_18 ;

  mat[0][0] = aux_1;
  mat[0][1] = aux_2;
  mat[0][2] = aux_2;
  mat[0][3] = aux_2;
  mat[0][4] = aux_2;
  mat[0][5] = aux_2;
  mat[1][0] = aux_2;
  mat[1][1] = aux_1;
  mat[1][2] = aux_2;
  mat[1][3] = aux_2;
  mat[1][4] = aux_2;
  mat[1][5] = aux_2;
  mat[2][0] = aux_2;
  mat[2][1] = aux_2;
  mat[2][2] = aux_1;
  mat[2][3] = aux_2;
  mat[2][4] = aux_2;
  mat[2][5] = aux_2;
  mat[3][0] = aux_2;
  mat[3][1] = aux_2;
  mat[3][2] = aux_2;
  mat[3][3] = 2*(-aux_4*aux_21*aux_18/2+aux_29+aux_28+aux_27-aux_4*aux_9*aux_11*aux_12+aux_4*aux_21*aux_23/2)*aux_25*aux_26-2*(-2*ma*aux_21*aux_18+aux_20+aux_19+2*aux_3*aux_17*aux_18+aux_16+aux_14+aux_13-aux_3*aux_9*aux_11*aux_12)*aux_22*aux_26;
  mat[3][4] = 2*(aux_37-ma*mb*aux_21*aux_18/2-2*aux_5*mc*aux_17*aux_18+aux_5*mc*aux_9*aux_11*aux_12-ma*mb*aux_9*aux_11*aux_12+ma*mb*aux_21*aux_23/2)*aux_25*aux_26-2*(aux_36+aux_35+aux_34+2*aux_4*mb*aux_17*aux_18+aux_33+aux_32+aux_31-aux_4*mb*aux_9*aux_11*aux_12)*aux_22*aux_26;
  mat[3][5] = 2*(aux_45+aux_44+aux_35+aux_32+aux_43+aux_42)*aux_25*aux_26-2*(2*mc*aux_21*aux_18-2*aux_39*aux_17*aux_18+aux_41+2*aux_4*mc*aux_17*aux_18+aux_40+aux_39*aux_9*aux_11*aux_12+aux_38-aux_4*mc*aux_9*aux_11*aux_12)*aux_22*aux_26;
  mat[4][0] = aux_2;
  mat[4][1] = aux_2;
  mat[4][2] = aux_2;
  mat[4][3] = -2*(aux_37+ma*mb*aux_21*aux_18/2+aux_47+aux_46+ma*mb*aux_9*aux_11*aux_12-ma*mb*aux_21*aux_23/2)*aux_48;
  mat[4][4] = -2*(aux_5*aux_21*aux_18/2+aux_29-aux_15*aux_11*aux_12+aux_27+aux_5*aux_9*aux_11*aux_12-aux_5*aux_21*aux_23/2)*aux_48;
  mat[4][5] = -2*(mb*mc*aux_21*aux_18/2+aux_49+aux_20+aux_14+mb*mc*aux_9*aux_11*aux_12-mb*mc*aux_21*aux_23/2)*aux_48;
  mat[5][0] = aux_2;
  mat[5][1] = aux_2;
  mat[5][2] = aux_2;
  mat[5][3] = 2*(aux_45+aux_44+aux_54+aux_43+aux_53+aux_42)*aux_51*aux_52-2*(2*ma*aux_21*aux_18+2*ma*aux_6*aux_17*aux_18+aux_19-2*aux_3*aux_17*aux_18+aux_16-ma*aux_6*aux_9*aux_11*aux_12+aux_13+aux_3*aux_9*aux_11*aux_12)*aux_50*aux_52;
  mat[5][4] = 2*(-mb*mc*aux_21*aux_18/2+aux_49-2*ma*aux_5*aux_17*aux_18-mb*mc*aux_9*aux_11*aux_12+ma*aux_5*aux_9*aux_11*aux_12+mb*mc*aux_21*aux_23/2)*aux_51*aux_52-2*(aux_36+2*mb*aux_6*aux_17*aux_18+aux_34+aux_54+aux_33-mb*aux_6*aux_9*aux_11*aux_12+aux_31+aux_53)*aux_50*aux_52;
  mat[5][5] = 2*aux_51*(-aux_6*aux_21*aux_18/2+aux_29+aux_28-aux_6*aux_9*aux_11*aux_12+aux_27+aux_6*aux_21*aux_23/2)*aux_52-2*aux_50*(-2*mc*aux_21*aux_18+2*aux_39*aux_17*aux_18+aux_41+aux_47+aux_40-aux_39*aux_9*aux_11*aux_12+aux_38+aux_46)*aux_52;
}

inline void manifoldZero2euler(Matrix6& mat, double manifold[6])
{
  //const double& mx = manifold[0];
  //const double& my = manifold[1];
  //const double& mz = manifold[2];
  const double& ma = manifold[3];
  const double& mb = manifold[4];
  const double& mc = manifold[5];

  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 , aux_12 ;
  aux_1 = 1 ;
  aux_2 = 0 ;
  aux_3 = mb*mc/4+ma/2 ;
  aux_4 = std::pow(ma,2) ;
  aux_5 = -std::pow(mb,2)/4 ;
  aux_6 = std::pow(mc,2) ;
  aux_7 = aux_6/4+aux_5-aux_4/4+1 ;
  aux_8 = 1/(std::pow(aux_7,2)+4*std::pow(aux_3,2)) ;
  aux_9 = 1/sqrt(1-4*std::pow(ma*mc/4-mb/2,2)) ;
  aux_10 = mc/2+ma*mb/4 ;
  aux_11 = -aux_6/4+aux_5+aux_4/4+1 ;
  aux_12 = 1/(std::pow(aux_11,2)+4*std::pow(aux_10,2)) ;

  mat[0][0] = aux_1;
  mat[0][1] = aux_2;
  mat[0][2] = aux_2;
  mat[0][3] = aux_2;
  mat[0][4] = aux_2;
  mat[0][5] = aux_2;
  mat[1][0] = aux_2;
  mat[1][1] = aux_1;
  mat[1][2] = aux_2;
  mat[1][3] = aux_2;
  mat[1][4] = aux_2;
  mat[1][5] = aux_2;
  mat[2][0] = aux_2;
  mat[2][1] = aux_2;
  mat[2][2] = aux_1;
  mat[2][3] = aux_2;
  mat[2][4] = aux_2;
  mat[2][5] = aux_2;
  mat[3][0] = aux_2;
  mat[3][1] = aux_2;
  mat[3][2] = aux_2;
  mat[3][3] = aux_7*aux_8+ma*aux_3*aux_8;
  mat[3][4] = mc*aux_7*aux_8/2+mb*aux_3*aux_8;
  mat[3][5] = mb*aux_7*aux_8/2-mc*aux_3*aux_8;
  mat[4][0] = aux_2;
  mat[4][1] = aux_2;
  mat[4][2] = aux_2;
  mat[4][3] = -mc*aux_9/2;
  mat[4][4] = aux_9;
  mat[4][5] = -ma*aux_9/2;
  mat[5][0] = aux_2;
  mat[5][1] = aux_2;
  mat[5][2] = aux_2;
  mat[5][3] = mb*aux_11*aux_12/2-ma*aux_10*aux_12;
  mat[5][4] = ma*aux_11*aux_12/2+mb*aux_10*aux_12;
  mat[5][5] = aux_11*aux_12+aux_10*mc*aux_12;
}

inline void eulerGradientXi(Matrix6& mat, const Vector6& e, const Vector6& xi, const Vector6& xj)
{
  //const double& ex  = e[0];
  //const double& ey  = e[1];
  //const double& ez  = e[2];
  const double& er  = e[3];
  const double& ep  = e[4];
  const double& eya = e[5];

  const double& x1 = xi[0];
  const double& y1 = xi[1];
  const double& z1 = xi[2];
  const double& a1 = xi[3];
  const double& b1 = xi[4];
  const double& c1 = xi[5];

  const double& x2 = xj[0];
  const double& y2 = xj[1];
  const double& z2 = xj[2];
  const double& a2 = xj[3];
  const double& b2 = xj[4];
  const double& c2 = xj[5];

  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 , aux_12 ,
         aux_13 , aux_14 , aux_15 , aux_16 , aux_17 , aux_18 , aux_19 , aux_20 , aux_21 , aux_22 , aux_23 ,
         aux_24 , aux_25 , aux_26 , aux_27 , aux_28 , aux_29 , aux_30 , aux_31 , aux_32 , aux_33 , aux_34 ,
         aux_35 , aux_36 , aux_37 , aux_38 , aux_39 , aux_40 , aux_41 , aux_42 , aux_43 , aux_44 , aux_45 ,
         aux_46 , aux_47 , aux_48 , aux_49 , aux_50 , aux_51 , aux_52 , aux_53 , aux_54 , aux_55 , aux_56 ,
         aux_57 , aux_58 , aux_59 , aux_60 , aux_61 , aux_62 , aux_63 , aux_64 , aux_65 , aux_66 , aux_67 ,
         aux_68 , aux_69 ;
  aux_1 = sin(c1) ;
  aux_2 = sin(a1) ;
  aux_3 = sin(ep) ;
  aux_4 = cos(a1) ;
  aux_5 = cos(ep) ;
  aux_6 = sin(eya) ;
  aux_7 = cos(c1) ;
  aux_8 = cos(b1) ;
  aux_9 = cos(eya) ;
  aux_10 = sin(b1) ;
  aux_11 = aux_4*aux_3-aux_2*aux_5*aux_6 ;
  aux_12 = aux_10*aux_11-aux_8*aux_5*aux_9 ;
  aux_13 = aux_2*aux_1+aux_4*aux_10*aux_7 ;
  aux_14 = aux_4*aux_10*aux_1-aux_2*aux_7 ;
  aux_15 = aux_4*aux_8*z2-aux_4*aux_8*z1+aux_14*y2-aux_14*y1+aux_13*x2+(-aux_2*aux_1-aux_4*aux_10*aux_7)*x1 ;
  aux_16 = aux_2*aux_10*aux_7-aux_4*aux_1 ;
  aux_17 = aux_4*aux_1-aux_2*aux_10*aux_7 ;
  aux_18 = -aux_2*aux_10*aux_1-aux_4*aux_7 ;
  aux_19 = -aux_2*aux_8*z2+aux_2*aux_8*z1+aux_18*y2-aux_18*y1+aux_17*x2+aux_16*x1 ;
  aux_20 = -aux_8*z2+aux_8*z1-aux_10*aux_1*y2+aux_10*aux_1*y1-aux_10*aux_7*x2+aux_10*aux_7*x1 ;
  aux_21 = -aux_4*aux_10*z2+aux_4*aux_10*z1+aux_4*aux_8*aux_1*y2-aux_4*aux_8*aux_1*y1+aux_4*aux_8*aux_7*x2-aux_4*aux_8*aux_7*x1 ;
  aux_22 = -aux_2*aux_10*z2+aux_2*aux_10*z1+aux_2*aux_8*aux_1*y2-aux_2*aux_8*aux_1*y1+aux_2*aux_8*aux_7*x2-aux_2*aux_8*aux_7*x1 ;
  aux_23 = aux_8*aux_7*y2-aux_8*aux_7*y1-aux_8*aux_1*x2+aux_8*aux_1*x1 ;
  aux_24 = aux_2*aux_10*aux_1+aux_4*aux_7 ;
  aux_25 = aux_16*y2-aux_16*y1+aux_18*x2+aux_24*x1 ;
  aux_26 = aux_2*aux_7-aux_4*aux_10*aux_1 ;
  aux_27 = aux_13*y2-aux_13*y1+aux_26*x2+aux_14*x1 ;
  aux_28 = sin(er) ;
  aux_29 = cos(er) ;
  aux_30 = aux_3*aux_28*aux_6+aux_29*aux_9 ;
  aux_31 = -aux_3*aux_28*aux_6-aux_29*aux_9 ;
  aux_32 = aux_2*aux_31-aux_4*aux_5*aux_28 ;
  aux_33 = aux_10*aux_32+aux_8*(aux_29*aux_6-aux_3*aux_28*aux_9) ;
  aux_34 = aux_3*aux_28*aux_9-aux_29*aux_6 ;
  aux_35 = aux_3*aux_29*aux_6-aux_28*aux_9 ;
  aux_36 = aux_28*aux_9-aux_3*aux_29*aux_6 ;
  aux_37 = aux_2*aux_36-aux_4*aux_5*aux_29 ;
  aux_38 = aux_10*aux_37+aux_8*(-aux_28*aux_6-aux_3*aux_29*aux_9) ;
  aux_39 = aux_28*aux_6+aux_3*aux_29*aux_9 ;
  aux_40 = 0 ;
  aux_41 = sin(a2) ;
  aux_42 = cos(b2) ;
  aux_43 = sin(b2) ;
  aux_44 = cos(c2) ;
  aux_45 = cos(a2) ;
  aux_46 = sin(c2) ;
  aux_47 = aux_41*aux_43*aux_44-aux_45*aux_46 ;
  aux_48 = aux_41*aux_43*aux_46+aux_45*aux_44 ;
  aux_49 = aux_14*aux_48+aux_13*aux_47+aux_4*aux_41*aux_8*aux_42 ;
  aux_50 = aux_41*aux_46+aux_45*aux_43*aux_44 ;
  aux_51 = aux_45*aux_43*aux_46-aux_41*aux_44 ;
  aux_52 = aux_14*aux_51+aux_13*aux_50+aux_4*aux_45*aux_8*aux_42 ;
  aux_53 = (aux_8*aux_1*aux_51+aux_8*aux_7*aux_50-aux_45*aux_10*aux_42)*aux_39+(aux_24*aux_51+aux_16*aux_50+aux_2*aux_45*aux_8*aux_42)*aux_35+aux_52*aux_5*aux_29 ;
  aux_54 = (aux_8*aux_1*aux_48+aux_8*aux_7*aux_47-aux_41*aux_10*aux_42)*aux_39+(aux_24*aux_48+aux_16*aux_47+aux_2*aux_41*aux_8*aux_42)*aux_35+aux_49*aux_5*aux_29 ;
  aux_55 = 1/(pow(aux_54,2)+pow(aux_53,2)) ;
  aux_56 = aux_42*aux_18*aux_46+aux_42*aux_17*aux_44+aux_2*aux_8*aux_43 ;
  aux_57 = aux_42*aux_14*aux_46+aux_42*aux_13*aux_44-aux_4*aux_8*aux_43 ;
  aux_58 = aux_42*aux_24*aux_46+aux_42*aux_16*aux_44-aux_2*aux_8*aux_43 ;
  aux_59 = aux_8*aux_42*aux_1*aux_46+aux_8*aux_42*aux_7*aux_44+aux_10*aux_43 ;
  aux_60 = 1/sqrt(1-pow(aux_59*aux_39+aux_58*aux_35+aux_57*aux_5*aux_29,2)) ;
  aux_61 = aux_4*aux_8*aux_42*aux_1*aux_46+aux_4*aux_8*aux_42*aux_7*aux_44+aux_4*aux_10*aux_43 ;
  aux_62 = aux_2*aux_8*aux_42*aux_1*aux_46+aux_2*aux_8*aux_42*aux_7*aux_44+aux_2*aux_10*aux_43 ;
  aux_63 = -aux_10*aux_42*aux_1*aux_46-aux_10*aux_42*aux_7*aux_44+aux_8*aux_43 ;
  aux_64 = aux_42*aux_13*aux_46+aux_42*aux_26*aux_44 ;
  aux_65 = aux_42*aux_16*aux_46+aux_42*aux_18*aux_44 ;
  aux_66 = aux_8*aux_42*aux_7*aux_46-aux_8*aux_42*aux_1*aux_44 ;
  aux_67 = aux_58*aux_5*aux_6+aux_59*aux_5*aux_9-aux_57*aux_3 ;
  aux_68 = aux_58*aux_30+aux_59*aux_34+aux_57*aux_5*aux_28 ;
  aux_69 = 1/(pow(aux_68,2)+pow(aux_67,2)) ;

  mat[0][0] =  aux_7*aux_12+aux_1*(aux_4*aux_5*aux_6+aux_2*aux_3);
  mat[0][1] = aux_1*aux_12+aux_7*(-aux_4*aux_5*aux_6-aux_2*aux_3);
  mat[0][2] = aux_8*aux_11+aux_10*aux_5*aux_9;
  mat[0][3] = aux_5*aux_6*aux_15-aux_3*aux_19;
  mat[0][4] = aux_5*aux_6*aux_22-aux_3*aux_21+aux_5*aux_9*aux_20;
  mat[0][5] = -aux_3*aux_27+aux_5*aux_6*aux_25+aux_5*aux_9*aux_23;
  mat[1][0] = aux_7*aux_33+aux_1*(aux_4*aux_30-aux_2*aux_5*aux_28);
  mat[1][1] = aux_1*aux_33+aux_7*(aux_4*aux_31+aux_2*aux_5*aux_28);
  mat[1][2] = aux_8*aux_32+aux_10*aux_34;
  mat[1][3] = aux_5*aux_28*aux_19+aux_30*aux_15;
  mat[1][4] = aux_30*aux_22+aux_5*aux_28*aux_21+aux_34*aux_20;
  mat[1][5] = aux_5*aux_28*aux_27+aux_30*aux_25+aux_34*aux_23;
  mat[2][0] = aux_7*aux_38+aux_1*(aux_4*aux_35-aux_2*aux_5*aux_29);
  mat[2][1] = aux_1*aux_38+aux_7*(aux_4*aux_36+aux_2*aux_5*aux_29);
  mat[2][2] = aux_8*aux_37+aux_10*aux_39;
  mat[2][3] = aux_5*aux_29*aux_19+aux_35*aux_15;
  mat[2][4] = aux_35*aux_22+aux_5*aux_29*aux_21+aux_39*aux_20;
  mat[2][5] = aux_5*aux_29*aux_27+aux_35*aux_25+aux_39*aux_23;
  mat[3][0] = aux_40;
  mat[3][1] = aux_40;
  mat[3][2] = aux_40;
  mat[3][3] = (aux_49*aux_35+(aux_18*aux_48+aux_17*aux_47-aux_2*aux_41*aux_8*aux_42)*aux_5*aux_29)*aux_53*aux_55-(aux_52*aux_35+(aux_18*aux_51+aux_17*aux_50-aux_2*aux_45*aux_8*aux_42)*aux_5*aux_29)*aux_54*aux_55;
  mat[3][4] = aux_53*((-aux_10*aux_1*aux_48-aux_10*aux_7*aux_47-aux_41*aux_8*aux_42)*aux_39+(aux_2*aux_8*aux_1*aux_48+aux_2*aux_8*aux_7*aux_47-aux_2*aux_41*aux_10*aux_42)*aux_35+(aux_4*aux_8*aux_1*aux_48+aux_4*aux_8*aux_7*aux_47-aux_4*aux_41*aux_10*aux_42)*aux_5*aux_29)*aux_55-((-aux_10*aux_1*aux_51-aux_10*aux_7*aux_50-aux_45*aux_8*aux_42)*aux_39+(aux_2*aux_8*aux_1*aux_51+aux_2*aux_8*aux_7*aux_50-aux_2*aux_45*aux_10*aux_42)*aux_35+(aux_4*aux_8*aux_1*aux_51+aux_4*aux_8*aux_7*aux_50-aux_4*aux_45*aux_10*aux_42)*aux_5*aux_29)*aux_54*aux_55;
  mat[3][5] = aux_53*((aux_8*aux_7*aux_48-aux_8*aux_1*aux_47)*aux_39+(aux_16*aux_48+aux_18*aux_47)*aux_35+(aux_13*aux_48+aux_26*aux_47)*aux_5*aux_29)*aux_55-((aux_8*aux_7*aux_51-aux_8*aux_1*aux_50)*aux_39+(aux_16*aux_51+aux_18*aux_50)*aux_35+(aux_13*aux_51+aux_26*aux_50)*aux_5*aux_29)*aux_54*aux_55;
  mat[4][0] = aux_40;
  mat[4][1] = aux_40;
  mat[4][2] = aux_40;
  mat[4][3] = -(aux_57*aux_35+aux_56*aux_5*aux_29)*aux_60;
  mat[4][4] = -(aux_63*aux_39+aux_62*aux_35+aux_61*aux_5*aux_29)*aux_60;
  mat[4][5] = -(aux_66*aux_39+aux_65*aux_35+aux_64*aux_5*aux_29)*aux_60;
  mat[5][0] = aux_40;
  mat[5][1] = aux_40;
  mat[5][2] = aux_40;
  mat[5][3] = aux_67*(aux_57*aux_30+aux_56*aux_5*aux_28)*aux_69-(aux_57*aux_5*aux_6-aux_56*aux_3)*aux_68*aux_69;
  mat[5][4] = aux_67*(aux_62*aux_30+aux_63*aux_34+aux_61*aux_5*aux_28)*aux_69-(aux_62*aux_5*aux_6+aux_63*aux_5*aux_9-aux_61*aux_3)*aux_68*aux_69;
  mat[5][5] = aux_67*(aux_65*aux_30+aux_66*aux_34+aux_64*aux_5*aux_28)*aux_69-(aux_65*aux_5*aux_6+aux_66*aux_5*aux_9-aux_64*aux_3)*aux_68*aux_69;
}

inline void eulerGradientXj(Matrix6& mat, const Vector6& e, const Vector6& xi, const Vector6& xj)
{
  //const double& ex  = e[0];
  //const double& ey  = e[1];
  //const double& ez  = e[2];
  const double& er  = e[3];
  const double& ep  = e[4];
  const double& eya = e[5];

  //const double& x1 = xi[0];
  //const double& y1 = xi[1];
  //const double& z1 = xi[2];
  const double& a1 = xi[3];
  const double& b1 = xi[4];
  const double& c1 = xi[5];

  //const double& x2 = xj[0];
  //const double& y2 = xj[1];
  //const double& z2 = xj[2];
  const double& a2 = xj[3];
  const double& b2 = xj[4];
  const double& c2 = xj[5];


  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 , aux_12 ,
         aux_13 , aux_14 , aux_15 , aux_16 , aux_17 , aux_18 , aux_19 , aux_20 , aux_21 , aux_22 , aux_23 ,
         aux_24 , aux_25 , aux_26 , aux_27 , aux_28 , aux_29 , aux_30 , aux_31 , aux_32 , aux_33 , aux_34 ,
         aux_35 , aux_36 , aux_37 , aux_38 , aux_39 , aux_40 , aux_41 , aux_42 , aux_43 , aux_44 , aux_45 ,
         aux_46 , aux_47 , aux_48 , aux_49 , aux_50 , aux_51 ;
  aux_1 = cos(a1) ;
  aux_2 = sin(b1) ;
  aux_3 = cos(c1) ;
  aux_4 = sin(a1) ;
  aux_5 = sin(c1) ;
  aux_6 = sin(ep) ;
  aux_7 = cos(b1) ;
  aux_8 = cos(ep) ;
  aux_9 = cos(eya) ;
  aux_10 = aux_4*aux_2*aux_3-aux_1*aux_5 ;
  aux_11 = sin(eya) ;
  aux_12 = aux_4*aux_2*aux_5+aux_1*aux_3 ;
  aux_13 = 0 ;
  aux_14 = aux_4*aux_5+aux_1*aux_2*aux_3 ;
  aux_15 = sin(er) ;
  aux_16 = cos(er) ;
  aux_17 = aux_1*aux_2*aux_5-aux_4*aux_3 ;
  aux_18 = cos(a2) ;
  aux_19 = cos(b2) ;
  aux_20 = sin(b2) ;
  aux_21 = cos(c2) ;
  aux_22 = sin(a2) ;
  aux_23 = sin(c2) ;
  aux_24 = aux_22*aux_23+aux_18*aux_20*aux_21 ;
  aux_25 = aux_18*aux_20*aux_23-aux_22*aux_21 ;
  aux_26 = aux_6*aux_16*aux_11-aux_15*aux_9 ;
  aux_27 = aux_15*aux_11+aux_6*aux_16*aux_9 ;
  aux_28 = (aux_7*aux_5*aux_25+aux_7*aux_3*aux_24-aux_18*aux_2*aux_19)*aux_27+(aux_12*aux_25+aux_10*aux_24+aux_4*aux_18*aux_7*aux_19)*aux_26+(aux_17*aux_25+aux_14*aux_24+aux_1*aux_18*aux_7*aux_19)*aux_8*aux_16 ;
  aux_29 = pow(aux_28,2) ;
  aux_30 = aux_22*aux_20*aux_21-aux_18*aux_23 ;
  aux_31 = aux_22*aux_20*aux_23+aux_18*aux_21 ;
  aux_32 = (aux_7*aux_5*aux_31+aux_7*aux_3*aux_30-aux_22*aux_2*aux_19)*aux_27+(aux_12*aux_31+aux_10*aux_30+aux_4*aux_22*aux_7*aux_19)*aux_26+(aux_17*aux_31+aux_14*aux_30+aux_1*aux_22*aux_7*aux_19)*aux_8*aux_16 ;
  aux_33 = 1/(pow(aux_32,2)+aux_29) ;
  aux_34 = aux_18*aux_23-aux_22*aux_20*aux_21 ;
  aux_35 = -aux_22*aux_20*aux_23-aux_18*aux_21 ;
  aux_36 = aux_22*aux_21-aux_18*aux_20*aux_23 ;
  aux_37 = -aux_20*aux_17*aux_23-aux_20*aux_14*aux_21-aux_1*aux_7*aux_19 ;
  aux_38 = -aux_20*aux_12*aux_23-aux_20*aux_10*aux_21-aux_4*aux_7*aux_19 ;
  aux_39 = -aux_7*aux_20*aux_5*aux_23-aux_7*aux_20*aux_3*aux_21+aux_2*aux_19 ;
  aux_40 = aux_19*aux_17*aux_23+aux_19*aux_14*aux_21-aux_1*aux_7*aux_20 ;
  aux_41 = aux_19*aux_12*aux_23+aux_19*aux_10*aux_21-aux_4*aux_7*aux_20 ;
  aux_42 = aux_7*aux_19*aux_5*aux_23+aux_7*aux_19*aux_3*aux_21+aux_2*aux_20 ;
  aux_43 = 1/sqrt(1-pow(aux_42*aux_27+aux_41*aux_26+aux_40*aux_8*aux_16,2)) ;
  aux_44 = aux_19*aux_17*aux_21-aux_19*aux_14*aux_23 ;
  aux_45 = aux_19*aux_12*aux_21-aux_19*aux_10*aux_23 ;
  aux_46 = aux_7*aux_19*aux_5*aux_21-aux_7*aux_19*aux_3*aux_23 ;
  aux_47 = aux_6*aux_15*aux_9-aux_16*aux_11 ;
  aux_48 = aux_6*aux_15*aux_11+aux_16*aux_9 ;
  aux_49 = aux_41*aux_48+aux_42*aux_47+aux_40*aux_8*aux_15 ;
  aux_50 = aux_41*aux_8*aux_11+aux_42*aux_8*aux_9-aux_40*aux_6 ;
  aux_51 = 1/(pow(aux_50,2)+pow(aux_49,2)) ;

  mat[0][0] =  aux_10*aux_8*aux_11+aux_7*aux_3*aux_8*aux_9+(-aux_4*aux_5-aux_1*aux_2*aux_3)*aux_6;
  mat[0][1] = aux_12*aux_8*aux_11+aux_7*aux_5*aux_8*aux_9+(aux_4*aux_3-aux_1*aux_2*aux_5)*aux_6;
  mat[0][2] = aux_4*aux_7*aux_8*aux_11-aux_2*aux_8*aux_9-aux_1*aux_7*aux_6;
  mat[0][3] = aux_13;
  mat[0][4] = aux_13;
  mat[0][5] = aux_13;
  mat[1][0] = (aux_10*aux_6*aux_15-aux_7*aux_3*aux_16)*aux_11+(aux_7*aux_3*aux_6*aux_15+aux_10*aux_16)*aux_9+aux_14*aux_8*aux_15;
  mat[1][1] = (aux_12*aux_6*aux_15-aux_7*aux_5*aux_16)*aux_11+(aux_7*aux_5*aux_6*aux_15+aux_12*aux_16)*aux_9+aux_17*aux_8*aux_15;
  mat[1][2] = (aux_4*aux_7*aux_6*aux_15+aux_2*aux_16)*aux_11+(aux_4*aux_7*aux_16-aux_2*aux_6*aux_15)*aux_9+aux_1*aux_7*aux_8*aux_15;
  mat[1][3] = aux_13;
  mat[1][4] = aux_13;
  mat[1][5] = aux_13;
  mat[2][0] = (aux_7*aux_3*aux_15+aux_10*aux_6*aux_16)*aux_11+((aux_1*aux_5-aux_4*aux_2*aux_3)*aux_15+aux_7*aux_3*aux_6*aux_16)*aux_9+aux_14*aux_8*aux_16;
  mat[2][1] = (aux_7*aux_5*aux_15+aux_12*aux_6*aux_16)*aux_11+((-aux_4*aux_2*aux_5-aux_1*aux_3)*aux_15+aux_7*aux_5*aux_6*aux_16)*aux_9+aux_17*aux_8*aux_16;
  mat[2][2] = (aux_4*aux_7*aux_6*aux_16-aux_2*aux_15)*aux_11+(-aux_4*aux_7*aux_15-aux_2*aux_6*aux_16)*aux_9+aux_1*aux_7*aux_8*aux_16;
  mat[2][3] = aux_13;
  mat[2][4] = aux_13;
  mat[2][5] = aux_13;
  mat[3][0] = aux_13;
  mat[3][1] = aux_13;
  mat[3][2] = aux_13;
  mat[3][3] = aux_29*aux_33-((aux_7*aux_5*aux_35+aux_7*aux_3*aux_34+aux_22*aux_2*aux_19)*aux_27+(aux_12*aux_35+aux_10*aux_34-aux_4*aux_22*aux_7*aux_19)*aux_26+(aux_17*aux_35+aux_14*aux_34-aux_1*aux_22*aux_7*aux_19)*aux_8*aux_16)*aux_32*aux_33;
  mat[3][4] = ((aux_22*aux_7*aux_19*aux_5*aux_23+aux_22*aux_7*aux_19*aux_3*aux_21+aux_22*aux_2*aux_20)*aux_27+(aux_22*aux_19*aux_12*aux_23+aux_22*aux_19*aux_10*aux_21-aux_4*aux_22*aux_7*aux_20)*aux_26+(aux_22*aux_19*aux_17*aux_23+aux_22*aux_19*aux_14*aux_21-aux_1*aux_22*aux_7*aux_20)*aux_8*aux_16)*aux_28*aux_33-((aux_18*aux_7*aux_19*aux_5*aux_23+aux_18*aux_7*aux_19*aux_3*aux_21+aux_18*aux_2*aux_20)*aux_27+(aux_18*aux_19*aux_12*aux_23+aux_18*aux_19*aux_10*aux_21-aux_4*aux_18*aux_7*aux_20)*aux_26+(aux_18*aux_19*aux_17*aux_23+aux_18*aux_19*aux_14*aux_21-aux_1*aux_18*aux_7*aux_20)*aux_8*aux_16)*aux_32*aux_33;
  mat[3][5] = aux_28*((aux_7*aux_3*aux_35+aux_7*aux_5*aux_30)*aux_27+(aux_10*aux_35+aux_12*aux_30)*aux_26+(aux_14*aux_35+aux_17*aux_30)*aux_8*aux_16)*aux_33-((aux_7*aux_3*aux_36+aux_7*aux_5*aux_24)*aux_27+(aux_10*aux_36+aux_12*aux_24)*aux_26+(aux_14*aux_36+aux_17*aux_24)*aux_8*aux_16)*aux_32*aux_33;
  mat[4][0] = aux_13;
  mat[4][1] = aux_13;
  mat[4][2] = aux_13;
  mat[4][3] = aux_13;
  mat[4][4] = -(aux_39*aux_27+aux_38*aux_26+aux_37*aux_8*aux_16)*aux_43;
  mat[4][5] = -(aux_46*aux_27+aux_45*aux_26+aux_44*aux_8*aux_16)*aux_43;
  mat[5][0] = aux_13;
  mat[5][1] = aux_13;
  mat[5][2] = aux_13;
  mat[5][3] = aux_13;
  mat[5][4] = aux_50*(aux_38*aux_48+aux_39*aux_47+aux_37*aux_8*aux_15)*aux_51-(aux_38*aux_8*aux_11+aux_39*aux_8*aux_9-aux_37*aux_6)*aux_49*aux_51;
  mat[5][5] = aux_50*(aux_45*aux_48+aux_46*aux_47+aux_44*aux_8*aux_15)*aux_51-(aux_45*aux_8*aux_11+aux_46*aux_8*aux_9-aux_44*aux_6)*aux_49*aux_51;
}

inline void propagateJacobianManifold(Matrix6& mat, const Vector6& xi, const Vector6& xj)
{
  //const double& x1 = xi[0];
  //const double& y1 = xi[1];
  //const double& z1 = xi[2];
  const double& a1 = xi[3];
  const double& b1 = xi[4];
  const double& c1 = xi[5];

  //const double& x2 = xj[0];
  //const double& y2 = xj[1];
  //const double& z2 = xj[2];
  const double& a2 = xj[3];
  const double& b2 = xj[4];
  const double& c2 = xj[5];

  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 , aux_12 , aux_13 ,
         aux_14 , aux_15 , aux_16 , aux_17 , aux_18 , aux_19 , aux_20 , aux_21 , aux_22 , aux_23 , aux_24 , aux_25 ,
         aux_26 , aux_27 , aux_28 , aux_29 , aux_30 ;
  aux_1 = cos(b1) ;
  aux_2 = cos(c1) ;
  aux_3 = sin(c1) ;
  aux_4 = sin(b1) ;
  aux_5 = 0 ;
  aux_6 = sin(a1) ;
  aux_7 = cos(a1) ;
  aux_8 = aux_6*aux_4*aux_2-aux_7*aux_3 ;
  aux_9 = aux_6*aux_4*aux_3+aux_7*aux_2 ;
  aux_10 = aux_6*aux_3+aux_7*aux_4*aux_2 ;
  aux_11 = aux_7*aux_4*aux_3-aux_6*aux_2 ;
  aux_12 = cos(a2) ;
  aux_13 = cos(b2) ;
  aux_14 = sin(b2) ;
  aux_15 = cos(c2) ;
  aux_16 = sin(a2) ;
  aux_17 = sin(c2) ;
  aux_18 = aux_11*(aux_12*aux_14*aux_17-aux_16*aux_15)+aux_10*(aux_16*aux_17+aux_12*aux_14*aux_15)+aux_7*aux_12*aux_1*aux_13 ;
  aux_19 = std::pow(aux_18,2) ;
  aux_20 = aux_16*aux_14*aux_15-aux_12*aux_17 ;
  aux_21 = aux_16*aux_14*aux_17+aux_12*aux_15 ;
  aux_22 = aux_11*aux_21+aux_10*aux_20+aux_7*aux_16*aux_1*aux_13 ;
  aux_23 = 1/(std::pow(aux_22,2)+aux_19) ;
  aux_24 = aux_13*aux_11*aux_17+aux_13*aux_10*aux_15-aux_7*aux_1*aux_14 ;
  aux_25 = -aux_16*aux_17-aux_12*aux_14*aux_15 ;
  aux_26 = aux_16*aux_15-aux_12*aux_14*aux_17 ;
  aux_27 = 1/sqrt(1-std::pow(aux_24,2)) ;
  aux_28 = aux_13*aux_9*aux_17+aux_13*aux_8*aux_15-aux_6*aux_1*aux_14 ;
  aux_29 = aux_1*aux_13*aux_3*aux_17+aux_1*aux_13*aux_2*aux_15+aux_4*aux_14 ;
  aux_30 = 1/(std::pow(aux_29,2)+std::pow(aux_28,2)) ;

  mat[0][0] = aux_1*aux_2;
  mat[0][1] = aux_1*aux_3;
  mat[0][2] = -aux_4;
  mat[0][3] = aux_5;
  mat[0][4] = aux_5;
  mat[0][5] = aux_5;
  mat[1][0] = aux_8;
  mat[1][1] = aux_9;
  mat[1][2] = aux_6*aux_1;
  mat[1][3] = aux_5;
  mat[1][4] = aux_5;
  mat[1][5] = aux_5;
  mat[2][0] = aux_10;
  mat[2][1] = aux_11;
  mat[2][2] = aux_7*aux_1;
  mat[2][3] = aux_5;
  mat[2][4] = aux_5;
  mat[2][5] = aux_5;
  mat[3][0] = aux_5;
  mat[3][1] = aux_5;
  mat[3][2] = aux_5;
  mat[3][3] = aux_19*aux_23-(aux_11*(-aux_16*aux_14*aux_17-aux_12*aux_15)+aux_10*(aux_12*aux_17-aux_16*aux_14*aux_15)-aux_7*aux_16*aux_1*aux_13)*aux_22*aux_23;
  mat[3][4] = -aux_24*aux_22*aux_23;
  mat[3][5] = (-aux_13*aux_11*aux_17-aux_13*aux_10*aux_15+aux_7*aux_1*aux_14)*aux_18*aux_23;
  mat[4][0] = aux_5;
  mat[4][1] = aux_5;
  mat[4][2] = aux_5;
  mat[4][3] = aux_5;
  mat[4][4] = -(aux_11*aux_26+aux_10*aux_25-aux_7*aux_12*aux_1*aux_13)*aux_27;
  mat[4][5] = -aux_22*aux_27;
  mat[5][0] = aux_5;
  mat[5][1] = aux_5;
  mat[5][2] = aux_5;
  mat[5][3] = aux_5;
  mat[5][4] = aux_29*(aux_9*aux_26+aux_8*aux_25-aux_6*aux_12*aux_1*aux_13)*aux_30-aux_28*(aux_1*aux_3*aux_26+aux_1*aux_2*aux_25+aux_12*aux_4*aux_13)*aux_30;
  mat[5][5] = aux_29*(aux_9*aux_21+aux_8*aux_20+aux_6*aux_16*aux_1*aux_13)*aux_30-aux_28*(aux_1*aux_3*aux_21+aux_1*aux_2*aux_20-aux_16*aux_4*aux_13)*aux_30;
}

inline void motionJacobianState(Matrix6& mat, const Vector6& xi, const Vector6& e)
{
  const double& ex  = e[0];
  const double& ey  = e[1];
  const double& ez  = e[2];
  //const double& er  = e[3];
  const double& ep  = e[4];
  const double& eya = e[5];

  //const double& x1 = xi[0];
  //const double& y1 = xi[1];
  //const double& z1 = xi[2];
  const double& a1 = xi[3];
  const double& b1 = xi[4];
  const double& c1 = xi[5];

  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 ,
         aux_12 , aux_13 , aux_14 , aux_15 , aux_16 , aux_17 , aux_18 , aux_19 , aux_20 , aux_21 ,
         aux_22 , aux_23 , aux_24 , aux_25 , aux_26;

  aux_1 = 1 ;
  aux_2 = 0 ;
  aux_3 = cos(a1) ;
  aux_4 = sin(b1) ;
  aux_5 = cos(c1) ;
  aux_6 = sin(a1) ;
  aux_7 = sin(c1) ;
  aux_8 = aux_6*aux_7+aux_3*aux_4*aux_5 ;
  aux_9 = cos(b1) ;
  aux_10 = -aux_6*aux_4*aux_7-aux_3*aux_5 ;
  aux_11 = sin(ep) ;
  aux_12 = std::pow(aux_9,2) ;
  aux_13 = cos(ep) ;
  aux_14 = cos(eya) ;
  aux_15 = sin(eya) ;
  aux_16 = std::pow(aux_6,2) ;
  aux_17 = std::pow(aux_13,2) ;
  aux_18 = -2*aux_3*aux_9*aux_4*aux_13*aux_11*aux_14 ;
  aux_19 = std::pow(aux_14,2) ;
  aux_20 = (2*aux_6*aux_9*aux_4*aux_17*aux_14+2*aux_3*aux_6*aux_12*aux_13*aux_11)*aux_15 ;
  aux_21 = aux_20+((aux_16+1)*aux_12-1)*aux_17*aux_19+aux_18+(1-2*aux_16)*aux_12*aux_17+(aux_16-1)*aux_12+1 ;
  aux_22 = 1/aux_21 ;
  aux_23 = std::pow(aux_3,2) ;
  aux_24 = std::pow(aux_15,2) ;
  aux_25 = ((aux_23-2)*aux_12+1)*aux_17*aux_24+aux_20+aux_18+((aux_23+1)*aux_12-1)*aux_17-aux_23*aux_12+1 ;
  aux_26 = 1/aux_25 ;

  mat[0][0] = aux_1;
  mat[0][1] = aux_2;
  mat[0][2] = aux_2;
  mat[0][3] = (aux_3*aux_7-aux_6*aux_4*aux_5)*ez+aux_8*ey;
  mat[0][4] = aux_3*aux_9*aux_5*ez+aux_6*aux_9*aux_5*ey-aux_4*aux_5*ex;
  mat[0][5] = (aux_6*aux_5-aux_3*aux_4*aux_7)*ez+aux_10*ey-aux_9*aux_7*ex;
  mat[1][0] = aux_2;
  mat[1][1] = aux_1;
  mat[1][2] = aux_2;
  mat[1][3] = aux_10*ez+(aux_3*aux_4*aux_7-aux_6*aux_5)*ey;
  mat[1][4] = aux_3*aux_9*aux_7*ez+aux_6*aux_9*aux_7*ey-aux_4*aux_7*ex;
  mat[1][5] = aux_8*ez+(aux_6*aux_4*aux_5-aux_3*aux_7)*ey+aux_9*aux_5*ex;
  mat[2][0] = aux_2;
  mat[2][1] = aux_2;
  mat[2][2] = aux_1;
  mat[2][3] = aux_3*aux_9*ey-aux_6*aux_9*ez;
  mat[2][4] = -aux_3*aux_4*ez-aux_6*aux_4*ey-aux_9*ex;
  mat[2][5] = aux_2;
  mat[3][0] = aux_2;
  mat[3][1] = aux_2;
  mat[3][2] = aux_2;
  mat[3][3] = (aux_6*aux_9*aux_4*aux_13*aux_15+aux_12*aux_13*aux_14-aux_3*aux_9*aux_4*aux_11)*aux_22;
  mat[3][4] = (aux_3*aux_13*aux_15+aux_6*aux_11)*aux_26;
  mat[3][5] = aux_2;
  mat[4][0] = aux_2;
  mat[4][1] = aux_2;
  mat[4][2] = aux_2;
  mat[4][3] = -(aux_3*aux_9*aux_13*aux_15+aux_6*aux_9*aux_11)/sqrt(aux_21);
  mat[4][4] = (aux_6*aux_4*aux_13*aux_15+aux_9*aux_13*aux_14-aux_3*aux_4*aux_11)/sqrt(aux_25);
  mat[4][5] = aux_2;
  mat[5][0] = aux_2;
  mat[5][1] = aux_2;
  mat[5][2] = aux_2;
  mat[5][3] = -(aux_6*aux_9*aux_17*aux_14*aux_15-aux_4*aux_17*aux_19-aux_3*aux_9*aux_13*aux_11*aux_14+aux_4)*aux_22;
  mat[5][4] = -(aux_3*aux_6*aux_9*aux_17*aux_24+((1-2*aux_23)*aux_9*aux_13*aux_11-aux_3*aux_4*aux_17*aux_14)*aux_15-aux_6*aux_4*aux_13*aux_11*aux_14+aux_3*aux_6*aux_9*aux_17-aux_3*aux_6*aux_9)*aux_26;
  mat[5][5] = aux_1;
}

inline void motionJacobianMeasurement(Matrix6& mat, const Vector6& xi, const Vector6& e)
{
  //const double& ex  = e[0];
  //const double& ey  = e[1];
  //const double& ez  = e[2];
  //const double& er  = e[3];
  const double& ep  = e[4];
  const double& eya = e[5];

  //const double& x1 = xi[0];
  //const double& y1 = xi[1];
  //const double& z1 = xi[2];
  const double& a1 = xi[3];
  const double& b1 = xi[4];
  const double& c1 = xi[5];

  double aux_1 , aux_2 , aux_3 , aux_4 , aux_5 , aux_6 , aux_7 , aux_8 , aux_9 , aux_10 , aux_11 ,
         aux_12 , aux_13 , aux_14 , aux_15 , aux_16 , aux_17 , aux_18 , aux_19 , aux_20 ;

  aux_1 = cos(b1) ;
  aux_2 = cos(c1) ;
  aux_3 = sin(a1) ;
  aux_4 = sin(b1) ;
  aux_5 = cos(a1) ;
  aux_6 = sin(c1) ;
  aux_7 = 0 ;
  aux_8 = cos(ep) ;
  aux_9 = std::pow(aux_1,2) ;
  aux_10 = sin(ep) ;
  aux_11 = cos(eya) ;
  aux_12 = std::pow(aux_11,2) ;
  aux_13 = std::pow(aux_3,2) ;
  aux_14 = (aux_13+1)*aux_9-1 ;
  aux_15 = sin(eya) ;
  aux_16 = (aux_13-1)*aux_9 ;
  aux_17 = std::pow(aux_8,2) ;
  aux_18 = (2*aux_3*aux_1*aux_4*aux_17*aux_11+2*aux_5*aux_3*aux_9*aux_8*aux_10)*aux_15+aux_14*aux_17*aux_12-2*aux_5*aux_1*aux_4*aux_8*aux_10*aux_11+(1-2*aux_13)*aux_9*aux_17+aux_16+1 ;
  aux_19 = 1/aux_18 ;
  aux_20 = 1/sqrt(aux_18) ;

  mat[0][0] = aux_1*aux_2;
  mat[0][1] = aux_3*aux_4*aux_2-aux_5*aux_6;
  mat[0][2] = aux_3*aux_6+aux_5*aux_4*aux_2;
  mat[0][3] = aux_7;
  mat[0][4] = aux_7;
  mat[0][5] = aux_7;
  mat[1][0] = aux_1*aux_6;
  mat[1][1] = aux_3*aux_4*aux_6+aux_5*aux_2;
  mat[1][2] = aux_5*aux_4*aux_6-aux_3*aux_2;
  mat[1][3] = aux_7;
  mat[1][4] = aux_7;
  mat[1][5] = aux_7;
  mat[2][0] = -aux_4;
  mat[2][1] = aux_3*aux_1;
  mat[2][2] = aux_5*aux_1;
  mat[2][3] = aux_7;
  mat[2][4] = aux_7;
  mat[2][5] = aux_7;
  mat[3][0] = aux_7;
  mat[3][1] = aux_7;
  mat[3][2] = aux_7;
  mat[3][3] = 1;
  mat[3][4] = -((aux_14*aux_8*aux_11-aux_5*aux_1*aux_4*aux_10)*aux_15-2*aux_3*aux_1*aux_4*aux_8*aux_12-aux_5*aux_3*aux_9*aux_10*aux_11+aux_3*aux_1*aux_4*aux_8)*aux_19;
  mat[3][5] = -(aux_5*aux_3*aux_9*aux_8*aux_15-aux_5*aux_1*aux_4*aux_8*aux_11+(aux_16+1)*aux_10)*aux_19;
  mat[4][0] = aux_7;
  mat[4][1] = aux_7;
  mat[4][2] = aux_7;
  mat[4][3] = aux_7;
  mat[4][4] = (aux_3*aux_1*aux_10*aux_15-aux_4*aux_10*aux_11+aux_5*aux_1*aux_8)*aux_20;
  mat[4][5] = -(aux_4*aux_8*aux_15+aux_3*aux_1*aux_8*aux_11)*aux_20;
  mat[5][0] = aux_7;
  mat[5][1] = aux_7;
  mat[5][2] = aux_7;
  mat[5][3] = aux_7;
  mat[5][4] = (aux_4*aux_15+aux_3*aux_1*aux_11)*aux_19;
  mat[5][5] = (aux_3*aux_1*aux_8*aux_10*aux_15-aux_4*aux_8*aux_10*aux_11+aux_5*aux_1*aux_17)*aux_19;
}

} // end namespace

#endif
