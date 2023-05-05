import rticonnextdds_connector as rti
from time import sleep
from os import path 

file_path = path.dirname(path.realpath(__file__))
print(file_path)
connector = rti.Connector("MyParticipantLibrary::MyPubParticipant", file_path+"\ShapeExample.xml")

# with rti.open_connector(
#         config_name="MyParticipantLibrary::MyPubParticipant",
#         url=file_path + "\ShapeExample.xml") as connector:

output = connector.get_output("MyPublisher::MyAircraftWriter")

# print("Waiting for subscriptions...")
# output.wait_for_subscriptions()

print("Writing...")
for i in range(1, 100):
    output.instance.set_dictionary({"position":[i+0.3515, i, i, i, i, i, i, i, i, i, i, i, i, i, i]})
    # output.instance.set_number("y", i*2)
    # output.instance.set_number("shapesize", 30)
    # output.instance.set_string("color", "BLUE")
    output.write()

    sleep(0.5) # Write at a rate of one sample every 0.5 seconds, for ex.

print("Exiting...")
output.wait() # Wait for all subscriptions to receive the data before exiting