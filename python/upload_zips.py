
import argparse, logging, os, grequests, shutil, glob, json

'''
1) Call Script with Parent directory name
2) Zip all folders in directory: make_archive(archive_name, 'zip', root_dir)
3) Upload all zip folders
'''



def respond(response, *args, **kwargs):
    #text = json.loads(response.text)
    if response.status_code == 201:
        file_logger.info("Resource created. Uploaded file.")
    else:
        file_logger.error("An error ocurred: {} Status Code {} ".format(response.text, response.status_code))

def main():
    # dest:  The object holds the argument values as attributes and can be accessed through
    #        e.g. args.dir (if name = 'dir')
    # store: Save the value, after optionally converting it to a different type.
    #        This is the default action taken if none is specified.

    parser = argparse.ArgumentParser("Welcome to the RoboDB interface!")
    parser.add_argument('dir', nargs='?', default=os.getcwd(), action='store',
                        help='Source directory for file upload, default is current directory')

    group = parser.add_argument_group('authentication')
    group.add_argument('-u', '--user', dest='u', action='store')
    group.add_argument('-p', '--password', dest='p', action='store')

    args = parser.parse_args()
    #print "Directory parent: ", args.dir


    # Logging setup
    URL = "http://robodb.cs.brown.edu:8001/api/files/"
    #URL = "http://127.0.0.1:8000/api/files/"
    DIR = args.dir
    USER = args.u
    PW = args.p
    LOG_FILENAME = 'uploaded.log'

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s %(levelname)-8s %(message)s',
                        datefmt='%a, %d %b %Y %H:%M:%S',
                        filename=LOG_FILENAME,
                        filemode='w')

    # Log to the console
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    logging.getLogger('').addHandler(console)

    # Log to file
    file_logger = logging.getLogger('Uploader')



    files = []
    # Find all directories, zip them and store their names
    for item in sorted(glob.glob(os.path.join(DIR,'*'))):
        full = os.path.join(DIR, item)
        if os.path.isdir(full) and not (os.path.exists(os.path.join(full + '.zip'))):
            zip_file = shutil.make_archive(full, 'zip', full)
            file_logger.info('Zipped file %s' % item)
            files.append(zip_file)
        else:
            logging.warning('Skipping file: %s' % full)

    print files


    # Asynchronous request to enable multiple file upload and resource creation
    rs = (grequests.post(URL, files={'datafile': open(file, 'rb')}, auth=(USER, PW), hooks=dict(response=respond))
                for file in files)

    grequests.map(rs)






if __name__ == "__main__":
    main()
