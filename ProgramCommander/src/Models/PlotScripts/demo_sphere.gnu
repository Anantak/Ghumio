# set terminal png transparent nocrop enhanced size 450,320 font "arial,8" 
# set output 'surface2.1.png'
reset
set view equal xyz
set dummy u, v
set key bmargin center horizontal Right noreverse enhanced autotitle nobox
set parametric
#set view 45, 50, 1, 1
#set isosamples 50, 10
#set hidden3d back offset 1 trianglepattern 3 undefined 1 altdiagonal bentover
set style data lines
#set ztics -1.00000,0.25,1.00000 norangelimit
set title "Parametric Sphere" 
set urange [ -1.57080 : 1.57080 ] noreverse nowriteback
set vrange [ 0.00000 : 6.28319 ] noreverse nowriteback

splot cos(u)*cos(v)*2+1, cos(u)*sin(v)*3, sin(u), \
cos(u)*cos(v)*2+10, cos(u)*sin(v)*3, sin(u)
