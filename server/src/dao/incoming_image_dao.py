class incoming_image_dao(DAO):

    def __init__(self, id, time_stamp, nanoseconds, image_path):
        self.id = id
        self.time_stamp = time_stamp
        self.nanoseconds = nanoseconds
        self.image_path = image_path
    def add_imcoming_image_to_db(self):
        
