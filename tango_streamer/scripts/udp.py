import decorators
import socket

class UDPhandle(decorators.Decorator):
    def decorate_func(self, func, *dec_args, **dec_kwargs):
        def decorator(*args, **kwargs):

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(('', dec_kwargs['port']))

            backlog = ""

            while True:
                data = sock.recv(65535)
                backlog += str(data)

                data_remains = True

                while data_remains:
                    start_loc = backlog.find(dec_kwargs['start_delim'])

                    if start_loc == -1:
                        data_remains = False
                        break
                    
                    backlog = backlog[start_loc:]
                    end_loc = backlog.find(dec_kwargs['end_delim'])

                    if end_loc == -1:
                        data_remains = False
                        break

                    pkt = backlog[len(dec_kwargs['start_delim']):end_loc]
                    backlog = backlog[end_loc + len(dec_kwargs['end_delim']):]

                    func(pkt=pkt, *args, **kwargs)
                    break
            sock.close()
                
        return decorator

