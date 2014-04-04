-- This file is run before each TCP shell is handled to a user and can be
-- used to define new commands.


function pos()
    x,y,a = position_get()
    print("("..x..";"..y..";"..a..")")
end

function encoder()
    for i = 1,6 do
        print(encoder_get(hexmotor, i))
    end
end

-- Finally greet the user if running in interactive mode
--

-- Finally greet the user if running in interactive mode
function greet()
    print("Wilkommen bei ReichOS 1.0 !")
    print("Research Embeded Interface for Communicating with Humans : Operating system")
    print("Die offizielle Scriptsprache des vierten Reichs !")
    print("---------------------------------------------------------------------------")
end

if __conn then
    greet()
end


