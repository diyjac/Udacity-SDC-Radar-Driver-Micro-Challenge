
(cl:in-package :asdf)

(defsystem "kvaser-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CANPacket" :depends-on ("_package_CANPacket"))
    (:file "_package_CANPacket" :depends-on ("_package"))
    (:file "CANData" :depends-on ("_package_CANData"))
    (:file "_package_CANData" :depends-on ("_package"))
  ))