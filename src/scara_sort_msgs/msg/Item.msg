# An item on the belt as the camera and the tracker see it
uint32 id           # given by the tracker, counts up
string colour       # red, green, blue, yellow or unknown
string shape        # cube, cylinder, prism or unknown
float64 x           # m, robot frame, where the item is now (predicted from the encoder)
float64 y
uint32 times_seen   # frames the camera saw it in; confirmed from 3
