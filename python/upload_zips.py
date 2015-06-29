#!/usr/bin/env python

import argparse, logging, os, grequests, shutil, json, yaml



'''
1) Call Script with Parent directory name: python upload_zips.py dir -u -p /*
2) Zip all folders in directory: make_archive(archive_name, 'zip', root_dir)
3) Upload all zip folders
'''
# HELPER METHODS
# Reformat json server response and dump to YAML
def reformat(resp, file_path, mode):
    orig_json = resp.json()
    resp_yml = []
    sub_items = {}

    for k,v in orig_json.iteritems():
        sub_items[k] = v
    
    resp_yml.append(sub_items)

    with open(file_path, mode) as output:
        yaml.safe_dump(resp_yml, output, default_flow_style=False)
    return output

# Loads a YAML file
def yaml_loader(file_path):
    with open(file_path, 'r') as stream:
        data = yaml.load(stream)
    return data

def find_and_zip(folder):
    files_to_upload = []
    thumbs_to_upload = []
    raw = False
    
    # Find thumbnail
    for item in os.listdir(folder):
        if item == "raw":
            raw = True
        path_thumb = os.path.join(os.getcwd(), folder, item)
        # Ignore hidden files
        if not item.startswith('.') and os.path.isfile(path_thumb) and path_thumb.lower().endswith('.png'):
            thumbs_to_upload.append(path_thumb)
            break
    if not thumbs_to_upload:
        raise ValueError("A thumbnail wasn't provided.")
    
    # Zip folder, temporarily ignore the /raw subfolder
    if raw:
        folder_n = os.path.basename(os.path.normpath(folder))
        dest_folder = os.path.join(os.getcwd(), folder_n + "_tmp")
        copied = shutil.copytree(folder, dest_folder, ignore=shutil.ignore_patterns( 'raw*'))
        folder = folder_n
        shutil.rmtree(dest_folder)
    
    zip_file = shutil.make_archive(folder, 'zip', folder)
    file_logger.info('Zipped file %s' % folder)
    files_to_upload.append(zip_file)


    return files_to_upload, thumbs_to_upload

#####################################################################################################################

def respond(response, *args, **kwargs):
    if response.status_code == 200:
        resp_dict = response.json()
        folder_name, ext = os.path.splitext(resp_dict['fname'])
        file_name = folder_name + '.yaml'
        path = os.path.join(folder_name,file_name)
        reformat(response, path, 'a')
        file_logger.info("Success. STATUS CODE {}{}".format(response.status_code, response.reason))
    elif response.status_code == 201: # Creating new resource
        resp_dict = response.json()
        print resp_dict
        folder_name, ext = os.path.splitext(resp_dict['fname'])
        file_logger.info("201 Resource created with ID {}. Uploaded file.".format(resp_dict['id']))
        file_name = folder_name + '.yaml'
        path = os.path.join(folder_name,file_name)
        reformat(response, path, 'w')

    else:
        file_logger.error("An error ocurred: STATUS CODE {} REASON: {}".format(response.status_code,
                                                            response.reason))

file_logger = None

def main():
    # dest:  The object holds the argument values as attributes and can be accessed through
    #        e.g. args.dir (if name = 'dir')
    # store: Save the value, after optionally converting it to a different type.
    #        This is the default action taken if none is specified.

    parser = argparse.ArgumentParser("Welcome to the RoboDB interface!")
    parser.add_argument('dir', nargs='+', default=os.getcwd(), action='store',
                        help='Source directory for file upload, default is current directory')

    group = parser.add_argument_group('authentication')
    group.add_argument('-u', '--user', dest='u', action='store')
    group.add_argument('-p', '--password', dest='p', action='store')

    args = parser.parse_args()


    # Logging setup
    URL = "http://robodb.cs.brown.edu:8001/api/files/"
    #URL = "http://127.0.0.1:8000/api/files/"
    #URL = "http://127.0.0.1:8000/api/files/2b87887937023b0a17098b8c09f88836/"
    
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
    global file_logger
    file_logger = logging.getLogger('Uploader')


    thumbnails, files, update_urls, file_updates, thumbnail_updates = ([] for i in range(5))
    cwd = os.getcwd()


    # Find all directories, zip them and store their names
    for full in DIR:
        # Get the dir name without / in order to find the yaml file
        dir_name = os.path.basename(os.path.normpath(full))
        yml_name = dir_name + '.yaml'
        yml_path = os.path.join(full, yml_name)
        # If YAML exists, this folder has already been uploaded and this will be a PUT request
        yml = os.path.exists(yml_path)
        
        # PUT: Update resource
        if yml and os.path.isdir(full):
            yml_content = yaml_loader(yml_path)
            id = yml_content[-1]['id']
            update_urls.append(URL + id + '/')
            print yml_path
        
            f_put, t_put = find_and_zip(full)
            file_updates += f_put
            thumbnail_updates += t_put
    
        # CREATE: Create Resource
        elif not yml and os.path.isdir(full):
            
            f_create, t_create = find_and_zip(full)
            files += f_create
            thumbnails += t_create

        else:
            logging.warning('Skipping file: %s' % full)



    # Asynchronous request to enable multiple file upload and resource creation
    if files and thumbnails:
        print "Creating Resources ", files
        rs = (grequests.post(URL, files={'datafile': open(file, 'rb'), 'image': open(image, 'rb')},
                             auth=(USER, PW), hooks=dict(response=respond)) for file, image in zip(files, thumbnails))
        grequests.map(rs)


    if update_urls and file_updates:
        print "Updating resources ", update_urls
        rs = (grequests.put(url, files={'datafile': open(file, 'rb'), 'image': open(image, 'rb')},
        auth=(USER, PW), hooks=dict(response=respond)) for url, file, image in zip(update_urls, file_updates, thumbnail_updates))
        
        grequests.map(rs)


    for f in files:
        print "Removing ", f
        os.remove(f)

    for u in file_updates:
        print "Removing ", u
        os.remove(u)

if __name__ == "__main__":
    main()
