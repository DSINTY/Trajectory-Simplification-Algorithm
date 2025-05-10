import os

directory = '../dataset/taxi_log_2008_by_id/'

n = 0
for file_name in os.listdir(directory):
    if not file_name.endswith(".txt"):
        continue
    # file_name = 'A000' + str(n)
    path = os.path.join(directory,file_name)

    f = open(path,'r')
    # make directory if not exist
    if not os.path.exists('../dataset/taxi_clean/'):
        os.makedirs('../dataset/taxi_clean/')
    output = open(os.path.join('../dataset/taxi_clean/',file_name),'w')
    for line in f.readlines():
            row = line.strip().split(',')
            output.write(row[2]+' '+row[3]+' '+row[1]+'\n')
            
    f.close()
    output.close()