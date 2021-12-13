import os
import os.path
import shutil

'''
    Create tmp folder for agent having id @agent_id
    Wipe content if already exists and @wipe equals True

    Folder will contain init files for agent and other tmp files 
    created and managed by @agent_id nodes during execution
'''
def create_tmp_folder_agent(agent_id, wipe = True):
    # remove tmp folder for agent if it does exist already
    if wipe and os.path.exists('/tmp/'+agent_id):
        shutil.rmtree('/tmp/'+agent_id)

    if not os.path.exists('/tmp/'+agent_id):
        # create fresh new tmp folder for agent
        os.mkdir('/tmp/'+agent_id)

'''
    Copy init file in tmp folder for agent
'''
def load_init_file(pathsource, init_filename, agent_id):
    if os.path.exists(pathsource):
        shutil.copyfile(pathsource, '/tmp/'+agent_id+'/'+init_filename)
    else:
        print(pathsource + '\t invalid path for init. belief/desire set file')