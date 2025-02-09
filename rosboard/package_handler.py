import os
import tornado
import tornado.web

from ament_index_python.packages import get_package_share_directory

class PackageFileHandler(tornado.web.RequestHandler):
    def initialize(self, node):
        self.node = node

    def get(self, path=None):
        # split the path into parts
        parts = path.split("/")
        if len(parts) < 2:
            self.set_status(404)
            self.set_header("Content-Type", "text/plain")
            self.write("Not found")
            self.finish()
            return
        package_dir = get_package_share_directory(parts[0])
        path = os.path.join(package_dir, *parts[1:])
        if not os.path.exists(path):
            self.set_status(404)
            self.set_header("Content-Type", "text/plain")
            self.write("Not found")
            self.finish()
            return

        self.set_status(200)
        self.set_header("Content-Type", "application/octet-stream")
        # read the file from the path
        with open(path, "rb") as f:
            self.write(f.read())
        self.finish()