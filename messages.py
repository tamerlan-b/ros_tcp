import json

class MsgType:
	GET = 0
	SEND = 1

class MsgDataType:
	POS = 0
	SWARM_BEST_POS = 1
	COMMENT = 2

class MsgPacker:
	# Pack message to send it over TCP
	@staticmethod
	def pack(msg):
	    msg_json = json.dumps(msg)
	    msg_json = msg_json.encode("utf-8")
	    return msg_json

	# Unpack message recceived through TCP
	@staticmethod
	def unpack(msg):
	    # Decode message
	    msg = msg.decode("utf-8")
	    # Parse json-string to dictionary
	    data = json.loads(msg)
	    return data
