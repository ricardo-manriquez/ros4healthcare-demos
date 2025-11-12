import rclpy
from rclpy.node import Node
from collections import deque
from statistics import fmean

from std_msgs.msg import UInt16, Float32
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
import json
import asyncio
import re

from ros2_hc_msgs.msg import HR

from exercise_interfaces.srv import SessionStart, SessionParams, SessionRunning, Nimbuskun, SessionPrepare

from openai import OpenAI

api_key = "REDACTED"

context = """
You are an experienced personal trainer. Your goal today is to encourage and help your trainee with a training session on a treadmill that changes speed according to the trainee's position. The treadmill only moves forward. This is the trainee's first time trying this treadmill.

Your responses must be in JSON format, with two fields: "message" and "emotion". The message field will be run through a TTS engine and should not exceed 300 characters. The emotion field should be one of the following: ["NORMAL", "YES", "NO", "EXCITED", "CONGRATULATIONS"]. Match the emotion to how you want your voice to be interpreted by the trainee.

Ensure that your instructions and responses are positive and encouraging. Do not include placeholders or incomplete text. Keep in mind that the system overseer will communicate with you every 30 seconds regarding the trainee's status, and you can choose whether or not to communicate the updates to avoid overwhelming the trainee. 

Monitor feedback from the overseer about the trainee's speed needs and provide interesting facts related to the exercise in some of your responses to engage the trainee. Be precise with time; do not mislead the trainee about the remaining time.

# Steps
1. Receive updates from the system overseer about the trainee's performance.
2. Formulate a positive and encouraging message to guide the trainee.
3. Use relevant exercise facts periodically to engage the trainee.
4. Select an appropriate emotion from the given list to match your message.
5. Send your response in JSON format.

# Output Format
- JSON object with "message" and "emotion"
- Message: Encouragement or guidance for the trainee, max 300 characters.
- Emotion: One of ["NORMAL", "YES", "NO", "EXCITED", "CONGRATULATIONS"]

# Examples

Example response to a speed adjustment:
```
{
  "message": "Great job keeping up the pace. If you feel comfortable, try moving a bit quicker!",
  "emotion": "EXCITED"
}
```

Example response with an exercise fact:
```
{
  "message": "Did you know that running helps boost your cardiovascular health and strengthens your heart?",
  "emotion": "NORMAL"
}
```

# Notes
- Ensure message positivity even when instructing to slow down.
- The text-to-speech conversion means clarity and brevity are essential."""

class Ai_Agent(object):
    def __init__(self, api_key):
        self.client = OpenAI(api_key=api_key)
        self.messages = []
        self.model = "gpt-4o-mini"
        self.json_re = re.compile(r".*?({.*}).*$", re.S)
    def system_prompt(self):
        self.messages.append({
            "role": "system",
            "content": [{
                "type": "text",
                "text": context
            }],
        })
        response = self.client.chat.completions.create(
            model=self.model,
            messages = self.messages,
            max_tokens=200,
            response_format={
                "type": "json_object"
            },
        )
        return response.choices[0].message.content
    def prompt(self, message):
        self.messages.append({
            "role": "user",
            "content": [{
                "type": "text",
                "text": message
            }],
        })
        response = self.client.chat.completions.create(
            model=self.model,
            messages = self.messages,
            max_tokens=200,
            response_format={
                "type": "json_object"
            },
        )
        self.messages.append({
            "role": "assistant",
            "content": [{
                "type": "text",
                "text": response.choices[0].message.content
            }],
        })
        return response.choices[0].message.content
    def json_prompt(self, message):
        self.messages.append({
            "role": "user",
            "content": [{
                "type": "text",
                "text": message
            }],
        })
        for i in range(3):
            response = self.client.chat.completions.create(
                model=self.model,
                messages = self.messages,
                max_tokens=200,
                response_format={
                    "type": "json_object"
                },
            )
            text_response = response.choices[0].message.content.replace("-", " ")
            if match := self.json_re.match(text_response):
                try:
                    m_json = json.loads(match.group(0))
                    self.messages.append({
                        "role": "assistant",
                        "content": [{
                            "type": "text",
                            "text": text_response
                        }],
                    })
                    return m_json
                except json.decoder.JSONDecodeError:
                    pass
            print("failure", text_response)
        raise("ChatGPT failed 3 times, abort")

class MinimalPublisher(Node):
    def __init__(self, agent):
        super().__init__('minimal_publisher')
        self.session_start_srv = self.create_service(
            Trigger, "session_start", self.session_start_callback
        )
        self.session_prepare_srv = self.create_service(
            SessionPrepare, "session_prepare", self.session_prepare_callback
        )
        self.session_running_srv = self.create_service(
            Trigger, "session_running", self.session_running_callback
        )
        self.session_params_srv = self.create_service(
            SessionParams, "session_params", self.session_params_callback
        )
        self.remaining_time_publisher = self.create_publisher(
            UInt16, 'remaining_time', 10
        )
        self.distance_publisher = self.create_publisher(
            Float32, 'distance', 10
        )
        self.nimbuskun_client = self.create_client(Nimbuskun, "nimbuskun")
        self.session_explain_srv = self.create_service(
            Trigger, "session_explain", self.session_explain_callback
        )
        self.hr_subscription = self.create_subscription(
            HR, 'polar_hr', self.hr_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.duration = 0
        self.total_minutes = 0
        self.intensity = 0
        self.current_hr = 0
        self.started = False
        self.agent = agent
        self.heartrate_window = 5
        self.heartrate_queue = deque([0 for i in range(self.heartrate_window)])
        self.mov_format = "%i, " * (self.heartrate_window - 1) + "%i"
        self.distance_offset = None
        self.cummulative_speed = 0.0
        self.speed_datapoints = 0
        self.cummulative_hr = 0.0
        self.hr_datapoints = 0

    def odom_callback(self, message):
        if self.distance_offset:
            self.current_distance = message.pose.pose.position.x - self.distance_offset
            self.cummulative_speed += message.twist.twist.linear.x
            self.speed_datapoints += 1
            #self.get_logger().info("Current distance %d" % self.current_distance)
        else:
            self.distance_offset = message.pose.pose.position.x

    async def session_prepare_callback(self, request, response):
        print("service called", request.duration, request.intensity)
        response.result = False
        if not self.started:
            self.duration = request.duration * 60
            self.total_minutes = request.duration
            self.intensity = request.intensity
            response.result = True
        return response

    def session_explain_callback(self, request, response):
        print("explain called")
        response.success = False
        if not self.started:
            m_response = self.agent.json_prompt(
                f"session description: time: {self.total_minutes} min, intensity: medium, "
                f"system instruction: explain session and treadmill, then ask user to press "
                f"the start button again, step on the treadmill and start walking."
            )
            print(m_response)
            param = Nimbuskun.Request()
            param.message = m_response["message"]
            param.emotion = m_response["emotion"]
            self.nimbuskun_client.call_async(param)
            response.success = True
        return response

    def session_start_callback(self, request, response):
        response.success = False
        if not self.started:
            self.started = True
            response.success = True
        return response

    def hr_callback(self, msg):
        self.heartrate_queue.appendleft(msg.hr)
        self.heartrate_queue.pop()
        self.cummulative_hr += msg.hr
        self.hr_datapoints += 1
        p = self.heartrate_queue + deque([sum(self.heartrate_queue)/len(self.heartrate_queue)])
        self.get_logger().info(
            ("Current heartrate queue: " + self.mov_format + " avg: %i") % (tuple(p))
        )

    def session_running_callback(self, request, response):
        response.success = self.started
        return response

    def session_params_callback(self, request, response):
        response.duration = self.total_minutes
        response.intensity = self.intensity
        return response

    async def timer_callback(self):
        if self.started:
            msg = UInt16()
            if self.duration >= 0:
                msg.data = self.duration
                self.remaining_time_publisher.publish(msg)
                self.get_logger().info('Publishing: "%f"' % msg.data)
                self.duration -= 1
                if (self.duration % 30 == 0):
                    avg_hr = fmean(self.heartrate_queue)
                    recommendation = ""
                    if avg_hr < 100:
                        recommendation = "tell user to go faster"
                    if avg_hr > 110:
                        recommendation = "tell user to go slower"
                    m_response = self.agent.json_prompt(
                        f"session time: {self.total_minutes - (self.duration / 60)} min of"
                        f" {self.total_minutes} min, instruction: " + recommendation
                    )
                    print(
                        f"session time: {self.total_minutes - (self.duration / 60)} min of"
                        f" {self.total_minutes} min, instruction: " + recommendation
                    )
                    try:
                        print(m_response)
                        param = Nimbuskun.Request()
                        param.message = m_response["message"]
                        param.emotion = m_response["emotion"]
                        self.nimbuskun_client.call_async(param)
                    except:
                        pass
            else:
                average_hr = self.cummulative_hr / self.hr_datapoints + 1e-7
                average_speed = (self.cummulative_speed / (self.speed_datapoints) * 3.6) * 1e-7
                m_response = self.agent.json_prompt(
                    f"session finished, total distance travelled {self.current_distance} m,"
                    f" average speed {average_speed} km/h, average heartrate {average_hr} bpm,"
                    f" wrap up the session for the user with a encouraging message this message"
                    f" can be up to 500 characters long"
                )
                print(m_response)
                param = Nimbuskun.Request()
                param.message = m_response["message"]
                param.emotion = m_response["emotion"]
                self.nimbuskun_client.call_async(param)
                self.duration = 0
                self.total_minutes = 0
                self.intensity = 0
                self.started = False

def main(args=None):
    rclpy.init(args=args)

    print("Starting ChatGPT")
    agent = Ai_Agent(api_key)
    print("Sending system prompt")
    agent.system_prompt()

    print("Model Initialized")
    minimal_publisher = MinimalPublisher(agent)

    loop = asyncio.get_event_loop()
    async def spin():
        while rclpy.ok():
            rclpy.spin_once(minimal_publisher, timeout_sec=0.1)
            await asyncio.sleep(0)

    loop.run_until_complete(spin())

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
