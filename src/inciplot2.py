import matplotlib.pyplot as plt

# Sample data
x = [-70, -70, -60, 60, 70, 70]
y = [0.3, 1, 2, 2, 1, 0.3]

# Create line plot
plt.plot(x, y, marker='o', linestyle='-', color='b')

# Set fixed size for x and y axes
plt.xlim(-90, 90)  # Setting x-axis limits from 0 to 6
plt.ylim(2, 0) # Setting y-axis limits from 0 to 12

# Set equal aspect ratio
#plt.gca().set_aspect('equal', adjustable='box')

# Add title and labels
plt.title("Speckle Visiblity on 1m wide glass pane")
plt.xlabel("Angle(degrees)")
plt.ylabel("Distance(meters)")

# Show plot
plt.show()
