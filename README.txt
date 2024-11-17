Hello! Welcome to my Assignment 1! My name is Anurag Pathak, my netID is anp185, and I am in section 5 of this class.
All methods are in Main.py. if you have ANY questions about how to run my code, please let me know.

I am going to tell you how to run this project. Near the bottom, you will see a comment which is indicating that
everything below it are methods that can be used to run the project (around line 520). I had use numpy in order to
generate my grids. This section is split into 2 parts. The first part is the visualization aspect of the grid. I have
provided an example set of methods that you can run to see the different visualizations of the grid.
For this mini visualization, all you need to is uncomment the parts that start with the line
"rows, cols = 10, 10" all the way to "visualize_grid(grid3)". If you run this code, it will generate a random 10 by 10
matrix, displays it, then runs all 3 a* algorithms ( in the order repeated forward, repeated backward, and adaptive) on
the grid. It will then display the path that the algorithm took to reach the goal. Also, just to note, the path that it
shows in the grind does not include all of the expansions that the algorithm made. It only shows the path that the
algorithm took to reach the goal. When running this program, you may notice that some of the packages/imports are not
installed on your laptop. Please pip install them and the program should work. If you have any questions, please let me
know.


Part 0: 50 101x101 grids
In the second part of the commented code, which I indicate near the very bottom of the Main.py, I have provided a method
to generate 50 101x101 grids. It will then save these grids to a folder called gridworlds. You can view these grid by using
a small snippet I included at the very bottom of the Main.py file. Above that snippet of code is the code where I use
a for loop to run through all 3 different algorithms on all 50 of the 101x101 grids. After the algoritm runs for all of
them, a average function will be called to display the average amount of expansions for each algorithm.

Part 1: N/A, all explanation is on the document

Part 2: If you want to test my code to see the different between prioritizing a small g value over a larger g value in
the case of a tie, you can uncomment one of my Node class defintions, and comment the other one. If you look at the top
of my Main.py file, you can see at the top of each node class defintion, I left a comment to indicate which one is which.

Part 3: N/A, all explanation is on the document

Part 4: N/A, all explanation is on the document

Part 5: N/A, all explanation is on the document



