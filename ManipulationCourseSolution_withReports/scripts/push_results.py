from git import Repo
import os
import csv

exercise_number = str(7)

with open('grading' + exercise_number + '.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        username = row[0]
        points = row[1]
        feedback = row[2]
        if len(points) == 0:
            continue
        print(username, points)

        repo_path = r'git@version.aalto.fi:' + username + \
            '_robotic_manipulation/exercise' + exercise_number + '.git'
        repo_dir = 'repos/' + username + '/exercise' + exercise_number
        commit_msg = r'Feedback and points added.'

        try:
            os.makedirs(repo_dir)
        except:
            pass

        if len(os.listdir(repo_dir)) == 0:
            Repo.clone_from(repo_path, repo_dir)

        repo = Repo(repo_dir)
        repo.remote().pull()

        f = open(repo_dir + "/feedback/points.txt", "w")
        f.write(str(points))
        f.close()

        f = open(repo_dir + "/feedback/feedback.txt", "w")
        f.write(feedback)
        f.close()

        repo.git.add(os.path.join(
            repo.working_tree_dir, "feedback/points.txt"))
        repo.git.add(os.path.join(
            repo.working_tree_dir, "feedback/feedback.txt"))

        repo.index.commit(commit_msg)
        repo.remote().push()
