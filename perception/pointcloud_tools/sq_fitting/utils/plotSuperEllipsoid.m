function plotSuperEllipsoid( a, b, c, e1, e2, filename ) 

  figure1 = figure()

  nxy = 20; nxz = nxy;
  i = 1;
  for w=-pi : 2*pi/nxy:pi;
    j = 1;
    for n=-pi/2:pi/nxz:pi/2;
      xA(j,i) = a*sign(cos(n))*abs(cos(n)).^e1*sign(cos(w))*abs(cos(w)).^e2;
      yA(j,i) = b*sign(cos(n))*abs(cos(n)).^e1*sign(sin(w))*abs(sin(w)).^e2;
      zA(j,i) = c*sign(sin(n))*abs(sin(n)).^e1;
      j = j+1;
    end
    i=i+1;
  end

  se = surf( xA, yA, zA, 'Parent',gca,...
	    'EdgeLighting','gouraud',...
	    'FaceLighting','gouraud',...
	    'LineWidth',1.5,...
	    'FaceColor',[0.68,0.92,1],...
	    'FaceAlpha',0.6,...
	    'EdgeColor',[0.04, 0.14, 0.42]);

  view([120,20]);
  axis equal; axis on; grid on;
  print( gcf, filename )
  

endfunction

% Local Variables:
% mode: octave
% End:
