
def moving_filter(data_input):
    score = []
    for i, raw_record in enumerate(data_input):
        if i == 0:
            score.append(raw_record)
        else:
            temp = []
            for tag_id, record in enumerate(raw_record):
                if record == 1:
                    temp.append(1)
                else:
                    if tag_id == 4:
                        left = tag_id - 1
                        right = tag_id + 1
                    elif tag_id == 0:
                        left = tag_id
                        right = tag_id + 2
                    else:
                        left = tag_id - 1
                        right = tag_id + 2

                    if 0 in data_input[i-1][left: right]:
                        temp.append(0)
                    else:
                        a = score[i-1][left: right] + raw_record[left: right]
                        temp.append(sum(a)/float(len(a)))
            score.append(temp)

    result = 0
    for data in score:
        result += (sum(data)/float(len(data)))

    return result/float(len(score))
