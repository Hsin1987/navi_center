import requests
import json
import rospy
import time, datetime
# from account_info import corp_id, corp_secret, agent_id
# corp_id = corp_id()
# corp_secret = corp_secret()
# agent_id = agent_id()

# Setting access_token log file
file_path = '/tmp/access_token.log'


class WXAlarm:
    def setting(self, param_dict):
        self.corp_id = param_dict['corp_id']
        self.corp_secret = param_dict['corp_secret']
        self.agent_id = param_dict['agent_id']
        self.access_token = self.gen_access_token()

    # Access token, use it when the saved token became invalid.
    def gen_access_token(self):
        get_token_url = 'https://qyapi.weixin.qq.com/cgi-bin/gettoken?corpid=%s&corpsecret=%s'\
                        % (self.corp_id, self.corp_secret)
        # print(get_token_url)
        r = requests.get(get_token_url)
        request_json = r.json()
        this_access_token = request_json['access_token']
        r.close()
        # Write the token into file
        try:
            f = open(file_path, 'w+')
            f.write(this_access_token)
            f.close()
        except Exception as e:
            rospy.logerror("[RSS] Generate Token Error: "+str(e))

        # Return the access_token
        return this_access_token

    def get_access_token_from_file(self):
        try:
            f = open(file_path, 'r+')
            this_access_token = f.read()
            f.close()
            return this_access_token
        except Exception as e:
            print(e)

    def sent(self, message,  to_user='@all'):
        flag = True
        while (flag):
            try:
                send_message_url = 'https://qyapi.weixin.qq.com/cgi-bin/message/send?access_token=%s' % self.access_token
                message_params = {
                    "touser": to_user,
                    "msgtype": "text",
                    "agentid": self.agent_id,
                    "text": {
                        "content": message
                    },
                    "safe": 0
                }
                r = requests.post(send_message_url, data=json.dumps(message_params))
                print('post success %s ' % r.text)

                # Determine whether sending is successful or not. If not, execute exception function.
                request_json = r.json()
                errmsg = request_json['errmsg']
                if errmsg != 'ok':
                    raise
                # If it's successful , change the flag.
                flag = False
            except Exception as e:
                print(e)
                self.access_token = self.gen_access_token()


def sent_rss_notification(rss_on, rss_notification, robot_id,  msg):
    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    if rss_on:
        rss_notification.sent(str(st) + " " + robot_id + " " + msg + " Request RSS.")

