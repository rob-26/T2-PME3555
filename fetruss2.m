## Copyright (C) 2025 rober
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.

## -*- texinfo -*-
## @deftypefn {} {@var{retval} =} fetruss2 (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: rober <rober@LAPTOP-RV3VKHDK>
## Created: 2025-05-17

function [k,m]=fetruss2(el,leng,area,rho,beta,ipt)

%--------------------------------------------------------------
%  Purpose:
%     Stiffness and mass matrices for the 2-d truss element
%     nodal dof {u_1 v_1 u_2 v_2}
%
%  Synopsis:
%     [k,m]=fetruss2(el,leng,area,rho,beta,ipt)
%
%  Variable Description:
%     k - element stiffness matrix (size of 4x4)
%     m - element mass matrix (size of 4x4)
%     el - elastic modulus
%     leng - element length
%     area - area of truss cross-section
%     rho - mass density (mass per unit volume)
%     beta - angle between the local and global axes                                 ipt = 1: consistent mass matrix
%            positive if the local axis is in the ccw direction from
%            the global axis
%     ipt = 1 - consistent mass matrix
%         = 2 - lumped mass matrix
%--------------------------------------------------------------------------

% stiffness matrix

 c=cos(beta); s=sin(beta);
 k= (area*el/leng)*[ c*c   c*s  -c*c  -c*s;...
                     c*s   s*s  -c*s  -s*s;...
                    -c*c  -c*s   c*c   c*s;...
                    -c*s  -s*s   c*s   s*s];


% consistent mass matrix

 if ipt==1

    m=(rho*area*leng/6)*[ 2*c*c+2*s*s  0  c*c+s*s  0;...
                          0  2*c*c+2*s*s  0  c*c+s*s;...
                          c*c+s*s  0  2*c*c+2*s*s  0;...
                          0  c*c+s*s  0  2*c*c+2*s*s];

% lumped mass matrix

 else

    m=(rho*area*leng/2)*[ c*c+s*s  0  0  0;...
                          0  c*c+s*s  0  0;...
                          0  0  c*c+s*s  0;...
                          0  0  0  c*c+s*s];

 end

