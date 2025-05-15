% Arquivo para simulação de passarela de pedestres como estrutura de treliça.
%¨
% Trabalho apresentado para disciplina PME3555
% Roberto Marques Matheo – 11734740
% Isabela Maria Mendes Lopes – 11261614
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc; %clf;

nel = 121;
nnel = 2;
ndof = 3;
nnode = 46;
edof = 6;
sdof = nnode * ndof;

%_______________________________________________________

%Criação da malha com coordenadas dos nós
tempsum = 0.0;
for i = 1:10
  ncoord(i,1) = tempsum; ncoord(i,2) = 0.0; ncoord(i,3) = 0.0;
  tempsum += 3.4644;
endfor

tempsum = 0.0;
for i = 11:20
  ncoord(i,1) = tempsum; ncoord(i,2) = 3.0; ncoord(i,3) = 0.0;
  tempsum += 3.4644;
endfor

tempsum = 1.732;
for i = 21:29
  ncoord(i,1) = tempsum; ncoord(i,2) = 0.0; ncoord(i,3) = 3.0;
  tempsum += 3.4644;
endfor

tempsum = 1.732;
for i = 30:38
  ncoord(i,1) = tempsum; ncoord(i,2) = 3.0; ncoord(i,3) = 3.0;
  tempsum += 3.4644;
endfor

tempsum = 3.4644
for i = 39:46
  ncoord(i,1) = tempsum; ncoord(i,2) = 1.5; ncoord(i,3) = 3.0;
  tempsum += 3.4644;
endfor

B = 300
%_______________________________________________________

% Propriedades dos elementos
elarea = zeros(nel, 1);
el_modulus = 205000000000;

%tipos 1 e 2
for j = [1:9, 28:44, 63:70]
  elarea(j) = 0.000001*4*(2*101.6+203.2)
endfor

%tipo 3
for j = [11:26, 46:61]
  elarea(j) = 0.000001*4*(2*101.6 + 152.4)
endfor

%tipo 4
for j = [10, 27, 45, 62]
  elarea(j) = 0.000001*6*(2*136.5+3*203.2)
endfor

%tipo 5
for j = [71, 80, 81, 89]
  elarea(j) = 0.000001*4*(2*101.6+152.4)
endfor

%tipo 6
for j = [72:79, 82:88, 90:121]
  elarea(j) = 0.000001*4*(50.8+50.8)
endfor


%_______________________________________________________

% Tabela de conectividade

nodes = [1,2; 2,3; 3,4; 4,5; 5,6;6,7; 7,8; 8,9; 9,10;
         1,21; 21,2; 2,22; 22,3; 3, 23; 23,4; 4,24; 24,5; 5,25; 25,6; 6,26; 26,7; 7,27; 27,8; 8,28;28,9; 9,29; 29,10;
         21,22; 22,23; 23,24; 24,25; 25,26; 26,27; 27,28; 28,29;
         11,12; 12,13; 13,14; 14,15; 15,16; 16,17; 17,18; 18,19; 19,20;
         11,30; 30,12; 12,31; 31,13; 13,32; 32,14; 14,33; 33,15; 15,34; 34,16; 16,35; 35,17; 17,36; 36,18; 18,37; 37,19; 19,38;38,20
         30,31; 31,32; 32,33; 33,34; 34,35; 35,36; 36,37; 37,38;
         1,11; 2,12; 3,13; 4,14; 5,15; 6,16; 7,17; 8,18; 9,19; 10,20;
         21,30; 22,31; 23,32; 24,33; 25,34; 26,35; 27,36; 28,37; 29,38;
         21, 39; 39,31; 30,39; 39,22; 22,40; 40,32; 31,40; 40,23; 23,41; 41,33; 32,41; 41,24; 24,42; 42,34; 33,42; 42,25;
         25,43; 43,35; 34,43; 43,26; 26,44; 44,36; 35,44; 44,27; 27,45; 45,37; 36,45; 45,28; 28,46; 46,38; 37,46; 46,29];

%_______________________________________________________

% Condições de contorno

bcdof = [1,2,3,28,29,30,31,32,33,58,59,60];
bcval = zeros(1,12);

%_______________________________________________________

% Inicialização das matrizes

ff = zeros(sdof, 1);                  % Vetor de forças
kk = zeros(sdof);                     % Matriz de rigidez global
index = zeros(nnel * ndof, 1);        % Vetor de índices
%elforce = zeros(nnel * ndof, 1);      % Vetor de forças do elemento
%eldisp = zeros(nnel * ndof, 1);       % Vetor de deslocamentos do elemento
%k = zeros(nnel * ndof, nnel * ndof);  % Matriz de rigidez do elemento

%_______________________________________________________

% Aplicação das forças nos nós

V_barras = 0.001*0.001*(18*3.464*4*(2*101.6+203.2) + 16*3.464*4*(2*101.6+203.2) +
                 32*3.464*4*(2*101.6+152.4) + 4*3.464*6*(2*136.5+3*203.2) +
                 4*3*4*(2*101.6+152.4) + 15*3*4*(50.8+50.8) + 32*2.121*4*(50.8+50.8));

rho_aco1020 = 7870;
struct_mass = V_barras * rho_aco1020;
struct_weight = 9.81 * struct_mass;

P_total = 374160+660690+struct_weight

conc_load = P_total /20;

ff(5:3:27) = -conc_load;
ff(35:3:57) = -conc_load;
%_______________________________________________________

% Iteração sobre cada elemento
teste_length = zeros(nel, 1);

for idx = 1:nel

  nd(1) = nodes(idx,1);
  nd(2) = nodes(idx,2);

  x1 = ncoord(nd(1),1); y1 = ncoord(nd(1),2); z1 = ncoord(nd(1),3);
  x2 = ncoord(nd(2),1); y2 = ncoord(nd(2),2); z2 = ncoord(nd(2),3);
  el_length = sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2);
  cx = (x2 - x1)/el_length; cy = (y2 - y1)/el_length;  cz = (z2 - z1)/el_length;


  area = elarea(idx);
  lambda = [cx^2, cx*cy, cx*cz;
            cx*cy, cy^2, cy*cz;
            cx*cz, cy*cz, cz^2]
  k_local = ((area * el_modulus)/el_length)*[lambda, -lambda; -lambda, lambda]

  counter1 = 0;
  for counter2 = 1:2
    start = (nd(counter2)-1)*3;
    for counter3 = 1:3
      counter1 = counter1 + 1;
      indice_elemento(counter1) = start + counter3;
    endfor
  endfor
  index = indice_elemento;

  for i=1:6
    ii = index(i);
    for j=1:6
      jj = index(j);
      kk(ii,jj) = kk(ii,jj) + k_local(i,j);
    endfor
  endfor

endfor

%_______________________________________________________

% Resolver matriz para obter vetor de deslocamentos

displac = zeros(121,1);
n = length(bcdof);
##
##desc = [4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121];
##dd1 = displac(desc);
##F11 = ff(desc);
##
##dd2 = displac(bcdof);
##F22 = ff(bcdof);
##
##k11 = kk(desc,desc)
##k12 = kk(desc, bcdof);
##k21 = kk(bcdof,desc);
##k22 = kk(bcdof,bcdof);
##
##dd1 = k11\(F11 - k12*dd2)
##F22 = k21*dd1 + k22*dd2

for i = 1:n
  c = bcdof(i);
  for j = 1:sdof
    kk(c, j) = 0;
    %kk(j, c) = 0;
  endfor
  kk(c, c) = 1;
  ff(c) = bcval(i)
endfor
##
##kk_inv = inv(kk)
##
##displac = kk_inv * ff

displac = kk\ff
reac = kk * displac - ff;
















