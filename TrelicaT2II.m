% Arquivo para simulação de passarela de pedestres como estrutura de treliça.
%¨
% Trabalho apresentado para disciplina PME3555
% Roberto Marques Matheo – 11734740
% Isabela Maria Mendes Lopes – 11261614
% Leonardo Tavares Valente – 12675913

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc; clf;

%Dados de entrada relevantes

nel = 121;              % Número de elementos
nnel = 2;               % Número de nós por elementos
ndof = 3;               % Graus de liberdade por nó
nnode = 46;             % Número de nós
edof = 6;               % Graus de liberdade por elemento
sdof = nnode * ndof;    % Graus de liberdade totais do sistema

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

tempsum = 1.7322;
for i = 21:29
  ncoord(i,1) = tempsum; ncoord(i,2) = 0.0; ncoord(i,3) = 3.0;
  tempsum += 3.4644;
endfor

tempsum = 1.7322;
for i = 30:38
  ncoord(i,1) = tempsum; ncoord(i,2) = 3.0; ncoord(i,3) = 3.0;
  tempsum += 3.4644;
endfor

tempsum = 3.4644;
for i = 39:46
  ncoord(i,1) = tempsum; ncoord(i,2) = 1.5; ncoord(i,3) = 3.0;
  tempsum += 3.4644;
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

% Propriedades dos elementos
el_modulus = 205000000000;

elleng = zeros(nel, 1);
elarea = zeros(nel, 1);
elvolume = zeros(nel, 1);

for iel=1:nel    %loop para encontrar comprimento de cada elemento

nd(1)=nodes(iel,1);   % 1º nó do i-ésimo elemento
nd(2)=nodes(iel,2);   % 2º nó do i-ésimo elemento

x1 = ncoord(nd(1),1); y1 = ncoord(nd(1),2); z1 = ncoord(nd(1),3);
x2 = ncoord(nd(2),1); y2 = ncoord(nd(2),2); z2 = ncoord(nd(2),3);

elleng(iel) = sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2);  % comprimento

end

%tipos 1 e 2
for j = [1:9, 28:44, 63:70]
  elarea(j) = 0.000001*4*(2*101.6+203.2);
  elvolume(j) = elleng(j) * elarea(j);
endfor

%tipo 3
for j = [11:26, 46:61]
  elarea(j) = 0.000001*4*(2*101.6 + 152.4);
  elvolume(j) = elleng(j) * elarea(j);
endfor

%tipo 4
for j = [10, 27, 45, 62]
  elarea(j) = 0.000001*6*(2*136.5+3*203.2);
  elvolume(j) = elleng(j) * elarea(j);
endfor

%tipo 5
for j = [71, 80, 81, 89]
  elarea(j) = 0.000001*4*(2*101.6+152.4);
  elvolume(j) = elleng(j) * elarea(j);
endfor

%tipo 6
for j = [72:79, 82:88, 90:121]
  elarea(j) = 0.000001*4*(50.8+50.8);
  elvolume(j) = elleng(j) * elarea(j);
endfor

for i = 1:121
  elprop(i,1) = el_modulus;
  elprop(i,2) = elarea(i);
endfor


%_______________________________________________________

% Condições de contorno

bcdof = [1,2,3,30,31,32,33,60];   % Graus de liberdade restritos
bcval = zeros(1,8);               % Restrição de deslocamento

%_______________________________________________________

% Inicialização das matrizes

ff = zeros(sdof, 1);                  % Vetor de forças
kk = zeros(sdof);                     % Matriz de rigidez global
index = zeros(nnel * ndof, 1);        % Vetor de índices
elforce = zeros(nnel * ndof, 1);      % Vetor de forças do elemento
eldisp = zeros(nnel * ndof, 1);       % Vetor de deslocamentos do elemento
k = zeros(nnel * ndof, nnel * ndof);  % Matriz de rigidez do elemento

%_______________________________________________________

% Carregamentos nodais

total_vol = sum(elvolume);
rho_aco1020 = 7850;
struct_mass = total_vol * rho_aco1020;
struct_weight = 9.81 * struct_mass;

f_total = 5000*31.18*3 + struct_weight;

f_conc = f_total/20;

ff(3:3:30) = -f_conc;
ff(33:3:60) = -f_conc;

%--------------------------
%  Loop para definição da matriz de rigidez
%--------------------------

for iel=1:nel    % loop for the total number of elements

nd(1)=nodes(iel,1);   % 1st connected node for the (iel)-th element
nd(2)=nodes(iel,2);   % 2nd connected node for the (iel)-th element

x1 = ncoord(nd(1),1); y1 = ncoord(nd(1),2); z1 = ncoord(nd(1),3);
x2 = ncoord(nd(2),1); y2 = ncoord(nd(2),2); z2 = ncoord(nd(2),3);

leng = elleng(iel);  % element length

cx = (x2 - x1)/leng; cy = (y2 - y1)/leng;  cz = (z2 - z1)/leng;

el=elprop(iel,1);               % extract elastic modulus
area=elprop(iel,2);             % extract cross-sectional area

index=feeldof(nd,nnel,ndof);  % extract system dofs for the element

k = fetruss3D(el,leng,area,cx,cy,cz); % compute element matrix

kk = feasmbl1(kk,k,index);           % assemble into system matrix

end

%---------------------------------------------------
%  apply constraints and solve the matrix
%---------------------------------------------------

[kk,ff]=feaplyc2(kk,ff,bcdof,bcval);  % apply the boundary conditions

disp = kk\ff;   % solve the matrix equation to find nodal displacements

forces = kk * disp;


%--------------------------------------------------
%  post computation for stress calculation
%--------------------------------------------------

for iel=1:nel         % loop for the total number of elements

nd(1)=nodes(iel,1);   % 1st connected node for the (iel)-th element
nd(2)=nodes(iel,2);   % 2nd connected node for the (iel)-th element

x1 = ncoord(nd(1),1); y1 = ncoord(nd(1),2); z1 = ncoord(nd(1),3);
x2 = ncoord(nd(2),1); y2 = ncoord(nd(2),2); z2 = ncoord(nd(2),3);

leng= sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2);  % element length

cx = (x2 - x1)/leng; cy = (y2 - y1)/leng;  cz = (z2 - z1)/leng;

el = elprop(iel,1);               % extract elastic modulus
area = elprop(iel,2);             % extract cross-sectional area

index = feeldof(nd,nnel,ndof);  % extract system dofs for the element

k = fetruss3D(el,leng,area,cx,cy,cz); % compute element matrix

for i=1:(nnel*ndof)           % extract displacements associated with
eldisp(i) = disp(index(i));     % (iel)-th element
end

elforce = k*eldisp;             % element force vector
stress(iel) = sqrt(elforce(1)^2+elforce(2)^2 + elforce(3)^2)/area; % stress calculation


if ((x2-x1)*elforce(4)) < 0;
stress(iel) = -stress(iel);
end

end
stress = stress';

for i = 1:121
  forcas_por_elemento(i) =  elarea(i) * stress(i) ;
end

forcas_por_elemento = forcas_por_elemento';

%% Visualiza��o de deslocamentos nodais

% Plot 2D
figure;
plot(ncoord(1:10,1), ncoord(1:10, 2), 'bo', 'DisplayName', 'Original');
hold on;
plot(ncoord(1:10,1)+disp(1:3:30), ncoord(1:10, 2)+ disp(2:3:30), 'ro', 'DisplayName', 'Displaced');
legend show;
xlabel('X Position');
ylabel('Z Position');
title('Nodal Displacements in 2D');
grid on;

% Plot 2D
figure;
plot(ncoord(1:20,1), ncoord(1:20, 2), 'bo', 'DisplayName', 'Original');
hold on;
plot(ncoord(1:20,1)+disp(1:3:60), ncoord(1:20, 2)+ disp(2:3:60), 'ro', 'DisplayName', 'Displaced');
legend show;
xlabel('X Position');
ylabel('Z Position');
title('Nodal Displacements in 2D');
grid on;

% Plot 2D
figure;
plot(ncoord(30:38,1), ncoord(30:38, 3), 'bo', 'DisplayName', 'Original');
hold on;
plot(ncoord(30:38,1)+disp(88:3:114), ncoord(30:38, 3)+ disp(89:3:114), 'ro', 'DisplayName', 'Displaced');
legend show;
xlabel('X Position');
ylabel('Z Position');
title('Nodal Displacements in 2D');
grid on;

## 3D Plot
##figure;
##plot3(node_positions(:, 1)/1e10, node_positions(:, 2)/1e10, node_positions(:, 3)/1e10, 'bo', 'DisplayName', 'Original');
##hold on;
##plot3(displaced_positions(:, 1)/1e10, displaced_positions(:, 2)/1e10, displaced_positions(:, 3)/1e10, 'ro', 'DisplayName', 'Displaced');
##legend show;
##xlabel('X Position');
##ylabel('Y Position');
##zlabel('Z Position');
##title('3D Visualization of Nodal Displacements');
##grid on;

% Plot da treliça com coloração baseada nas forças nos elementos
figure;
hold on;
colormap(jet);
caxis([-max(abs(forcas_por_elemento)), max(abs(forcas_por_elemento))]);
colorbar;
title('Treliça colorida segundo forças axiais');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;

% Loop pelos elementos para plotar as barras com cor
for i = 1:nel
    n1 = nodes(i, 1);
    n2 = nodes(i, 2);

    x = [ncoord(n1, 1), ncoord(n2, 1)];
    y = [ncoord(n1, 2), ncoord(n2, 2)];
    z = [ncoord(n1, 3), ncoord(n2, 3)];

    % Normaliza força para cor
    f = forcas_por_elemento(i);
    color_value = (f - (-max(abs(forcas_por_elemento)))) / (2*max(abs(forcas_por_elemento))); % [0,1]
    cor = interp1(linspace(0,1,64), jet(64), color_value);

    plot(x, z, 'Color', cor, 'LineWidth', 2 + 3 * abs(f)/max(abs(forcas_por_elemento)));
end







