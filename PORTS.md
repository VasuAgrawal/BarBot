Global Localization:
    Listens for data from beacons on port 8888
    Sends it to the scheduler

Scheduler:
    Listens for connections on port 4242
    Receives data from Global Localization
    Forwards location and waypoints to the robots
    Talks to postgres database

Ordering System:
    Serves webpages from port 8080
    Talks to postgres database

Postgres Database:
    Listens to connections on port 10601
