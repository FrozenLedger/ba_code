from std_srvs.srv import TriggerRequest, TriggerResponse

class ROSTriggerHandler:
    """A callable class that handles the execution of an injected callable and executes it when called by a ros service request of type std_srvs.srv.TriggerRequest.
    The call returns a std_srvs.srv.TriggerResponse with <success> and <message> fields."""
    def __init__(self, hook, message=""):
        self._hook = hook
        self._message = message

    def __call__(self, request: TriggerRequest) -> TriggerResponse:
        try:
            self._hook()
            return TriggerResponse(success=True,message=self._message)
        except Exception as e:
            print(e)
            return TriggerResponse(success=False,message=str(e))