  for i=1:6
    ii = index(i)
    for j=1:6
      jj = index(j)
      kk(ii,jj) = kk(ii,jj) + k_local(i,j)
    endfor
  endfor

