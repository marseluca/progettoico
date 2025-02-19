% Questo script converte una mappa 3d in formato "wrl" in una matlab figure

close all
scalefactor = 1000;
import_data_catia_vrml('MAP.wrl',scalefactor);
view(0,90);


savefig("MAP.fig")