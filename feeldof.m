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
## @deftypefn {} {@var{retval} =} feeldof (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: rober <rober@LAPTOP-RV3VKHDK>
## Created: 2025-05-17

function [index]=feeldof(nd,nnel,ndof)
%----------------------------------------------------------
%  Purpose:
%     Compute system dofs associated with each element
%
%  Synopsis:
%     [index]=feeldof(nd,nnel,ndof)
%
%  Variable Description:
%     index - system dof vector associated with element "iel"
%     iel - element number whose system dofs are to be determined
%     nnel - number of nodes per element
%     ndof - number of dofs per node
%-----------------------------------------------------------

 edof = nnel*ndof;
   k=0;
   for i=1:nnel
     start = (nd(i)-1)*ndof;
       for j=1:ndof
         k=k+1;
         index(k)=start+j;
       end
   end
