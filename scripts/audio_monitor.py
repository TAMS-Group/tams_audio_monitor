#!/usr/bin/env python

import rospy

from sound_play.msg import SoundRequest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class AudioMonitor:
    def __init__(self):
        self.last_audio = rospy.Time()
        self.pub = rospy.Publisher('robotsound', SoundRequest, queue_size= 1, tcp_nodelay= True)

        self.sub_diag = rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.diag_cb, queue_size= 1)

    def send_sound(self, request):
        now = rospy.Time.now()
        if now - self.last_audio > rospy.Duration(30.0):
            self.pub.publish(request)
            self.last_audio = now

    def diag_cb(self, msg):
        for s in msg.status:
            if s.level == DiagnosticStatus.ERROR:
                self.send_sound(SoundRequest(
                    sound= SoundRequest.BACKINGUP, # abuse sound id because it sounds best for this. we really want our own sound theme here.
                    volume= 1.0,
                    command= SoundRequest.PLAY_ONCE))

if __name__ == '__main__':
  rospy.init_node('audio_monitor')

  monitor= AudioMonitor()
  rospy.spin()
