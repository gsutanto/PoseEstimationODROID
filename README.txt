to run the program:

% ./main 1 1 [image_file]

the first parameter is: 1 print out translations, 0 no print

the second parameter is: 1 displaying image, 0 no display

the third option is a image file

Output Format:
==============================================================
rvec (roation vector): [theta, phi, psi]
rmat (rotation matrics): [3 x 3]

tvec (translation vector): [x, y, z]

transfvec12 (a combination of rmat tvec): [ rmat | tvec ] ^ {t}
===============================================================
When there is no detection, all numbers are "1e+21".

