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
## @deftypefn {} {@var{retval} =} fetruss3D (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: rober <rober@LAPTOP-RV3VKHDK>
## Created: 2025-05-17

function k =fetruss3D(el,leng,area,cx,cy,cz)

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

lambda = [cx^2, cx*cy, cx*cz;
          cx*cy, cy^2, cy*cz;
          cx*cz, cy*cz, cz^2];

k = (area*el/leng)*[lambda, -lambda; -lambda, lambda];



 end
