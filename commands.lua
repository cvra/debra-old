-- Displays a welcome message to the user.
function greet()
    print("Wilkommen bei ReichsLua 1.0 !")
    print("Die offizielle Scriptsprache des vierten Reichs !")
    print("-------------------------------------------------")
end

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


if __conn then
    greet()
end


