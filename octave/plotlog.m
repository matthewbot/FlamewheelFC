function plotlog(log, T)
  R = rows(log);
  if R == 4
    colors = "krgb";
  else
    colors = "rgbmck";
  endif

  figure();
  hold on;
  for i=1:rows(log)
    plot(T, log(i, :), colors(i));
  endfor
  hold off;
endfunction