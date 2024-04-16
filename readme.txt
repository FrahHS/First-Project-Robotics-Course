// Print datalog coordinates
        Vector ecef = gps_to_ecef(latitude, longitude, altitude);
        ROS_INFO(
            "\n--------------------------------\n"
            "-HEADER:\n"
            "seq: %i\n"
            "stamp: %i\n"
            "-GPS:\n"
            "latitude: %f, longitude: %f, altitude: %f\n"
            "-ECEF:\n"
            "x: [%f], y: [%f], z: [%f]\n"
            "-ENU:\n"
            "x: [%f], y: [%f], z: [%f]\n"
            "--------------------------------\n",
            msg->header.seq,
            msg->header.stamp.sec,
            msg->latitude,
            msg->longitude,
            msg->altitude,
            ecef[0],
            ecef[1],
            ecef[2],
            x,
            y,
            z
        );
