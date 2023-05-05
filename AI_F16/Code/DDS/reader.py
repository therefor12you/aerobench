import rticonnextdds_connector as rti
from os import path 

file_path = path.dirname(path.realpath(__file__))
print(file_path)
with rti.open_connector(
        config_name="MyParticipantLibrary::MySubParticipant",
        url=file_path + "\ShapeExample.xml") as connector:

    input = connector.get_input("MySubscriber::MyAircraftReader")

    print("Waiting for publications...")
    input.wait_for_publications() # wait for at least one matching publication

    print("Waiting for data...")
    for i in range(1, 500):
        input.wait() # wait for data on this input
        input.take()
        for sample in input.samples.valid_data_iter:
            # You can get all the fields in a get_dictionary()
            data = sample.get_dictionary()
            x = data['position']
            # y = data['y']

            # Or you can access the field individually
            # size = sample.get_number("shapesize")
            # color = sample.get_string("color")
            print("Received x: " + repr(x))