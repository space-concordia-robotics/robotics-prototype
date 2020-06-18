# build
`docker build -t my-ros-k-app .`

# Run
`docker run -p 5000:5000 -p 9090:9090 -it --rm --name my-running-app -v scvolume:/home/scuser/Programming/robotics-prototype my-ros-k-app`

# Open another terminal of running image
`docker exec -it my-running-app bash`

Now you should be able to open localhost:5000 and be greeted with a working base-station page.

