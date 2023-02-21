CS396 Artificial Life - Jadon Lau
----------------------------------
Running the Code:

1.) Clone the repository

2.) Run the search.py file in the folder

Assignment 7 - Expanding the random creature generator to 3D
----------------------------------
Here instead of the 1 dimensional snake we did in Assignment 6, I created a random creature generator to work in the x, y, and z plane. To do this, I created links up until a certain number of links, then sent cubes (which are rectangular prisms) to pryosim in order to simulate a body part. After the link is sent, I then generated joints to create a connection between the links. The joint placement depends on the axis selected and are randomized using the random module. 


![Untitled Notebook 5-1](https://user-images.githubusercontent.com/98376049/220240861-147931a1-0cf9-4b29-b33d-3ce197868b93.png)
