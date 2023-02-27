Author: Muhammad Shahab 
Github: https://github.com/MuhamdShahab/
Linkedin: https://www.linkedin.com/in/muhammad-shahab-ph923165051365/

The following Algorithm is an implementation of the A star Algorithm on ESP32. Which uses the piority Queue and a simple Queue. 
Lets dive into the functions:
1. User can create a 2D array of own choice using the function 2D array-->int** array = getmap(int param, int m, int n) which return the array of n rows and m columns
and the all the elements at those position will be initialized with the param value;
2. After that user can print the array using the function printmap(2D array, int m, int n) which is a void type. the array which will
get printed till m columns and n rows passed to a function.
3. A star algorithm get implemented with obstacles as well user can place obstacles using the place_obstacle(2D array, col, row). it will put 0 over that place in 
map and updated map with zero will be returned.
4. Astar function return a 2D array with 2 rows(x and y) of the coordinates and the number of the column by getsize() function and getpath() 
gives the coordinated array.
5. if you want to calcualte the path with in the range you can give the left and the right lim in the A star function.
