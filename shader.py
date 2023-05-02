
class myShader():
    def __init__(self):
        self.v_shader='''#version 140

            struct p3d_LightSourceParameters {
                vec4 color;
                vec3 spotDirection; 
                sampler2DShadow shadowMap; 
                mat4 shadowMatrix;
            }; 

            uniform p3d_LightSourceParameters my_light;
            uniform mat4 p3d_ModelViewProjectionMatrix;
            uniform mat3 p3d_NormalMatrix;

            in vec4 p3d_Vertex;
            in vec3 p3d_Normal;
            in vec2 p3d_MultiTexCoord0;

            out vec2 uv;
            out vec4 shadow_uv;
            out vec3 normal;

            void main() {
                //position    
                gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;      
                //normal      
                normal = p3d_NormalMatrix * p3d_Normal;
                //uv
                uv = p3d_MultiTexCoord0;
                //shadows
                shadow_uv = my_light.shadowMatrix * p3d_Vertex;
            }
        '''
        #fragment shader
        self.f_shader='''#version 140

            struct p3d_LightSourceParameters {
                vec4 color;
                vec3 spotDirection; 
                sampler2DShadow shadowMap; 
                mat4 shadowMatrix;
            }; 

            uniform p3d_LightSourceParameters my_light;
            uniform sampler2D p3d_Texture0;
            uniform vec3 camera_pos;
            uniform float shadow_blur;

            in vec2 uv;
            in vec4 shadow_uv;
            in vec3 normal;

            out vec4 color;

            void main() {
                //base color
                vec3 ambient=vec3(0.1, 0.1, 0.2);    
                //texture        
                vec4 tex=texture(p3d_Texture0, uv);        
                //light ..sort of, not important
                vec3 light=my_light.color.rgb*max(dot(normalize(normal),-my_light.spotDirection), 0.0);
                
                //shadows
                float shadow= textureProj(my_light.shadowMap,shadow_uv); //meh :|
                
                //make the shadow brighter
                shadow=0+shadow*0.9;
                
                color=vec4(tex.rgb*(light*shadow+ambient), tex.a);
                
            }
        '''                    

