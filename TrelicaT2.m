% Arquivo para simulação de passarela de pedestres como estrutura de treliça.
%¨
% Trabalho apresentado para disciplina PME3555
% Roberto Marques Matheo – 11734740
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc; clf;

nel = 123;
nnel = 2;
ndof = 3;
nnode = 38;
sdof = nnode * ndof;

%ncoord(1,1) = 0.0; ncoord(1,2) = 0.0; ncoord(1,3) = 0.0;
%ncoord(2,1) = 3.464; ncoord(2,2) = 0.0; ncoord(2,3) = 0.0;
%ncoord(3,1) = 6.928; ncoord(3,2) = 0.0; ncoord(3,3) = 0.0;
%ncoord(3,1) = 6.928; ncoord(3,2) = 0.0; ncoord(3,3) = 0.0;

tempsum = 0.0
for i = 1:10

  ncoord(i,1) = tempsum; ncoord(i,2) = 0.0; ncoord(i,3) = 0.0;
  tempsum += 3.464;
endfor

ncoord
sdof
