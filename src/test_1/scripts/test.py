#coding=utf-8

import transforms3d

if len(t)>0:
      try:
        xck = interpolate.splrep(t, x)
        yck = interpolate.splrep(t, y)
        vck = interpolate.splrep(t, v)
        thetack = interpolate.splrep(t, theta)
        omegack = interpolate.splrep(t, omega)
        mat=transforms3d.quaternions.quat2mat([pos.orientation.x,pos.orientation.y,pos.orientation.z,pos.orientation.w])
        euler=transforms3d.euler.mat2euler(mat)[0]
        time_from_start = rospy.get_rostime().to_sec()-stamp.to_sec()
        time_ref=time_from_start+look_ahead
        x_n_t=interpolate.splev(time_ref, xck, der=0) # Refer to the init pose
        y_n_t=interpolate.splev(time_ref, yck, der=0) # Refer to the init pose
        v_n_t=interpolate.splev(time_ref, vck, der=0)
        theta_n_t=interpolate.splev(time_ref, thetack, der=0)
        omega_n_t=interpolate.splev(time_ref, omegack, der=0)
      except:
        x_n_t=x[1]
        y_n_t=y[1]
        v_n_t=v[1]
        theta_n_t=theta[1]
        omega_n_t=omega[1]
      x0=pos.position.x-INIT_POSITION[0] # Both coordinate take the initial point as the origin
      y0=pos.position.y-INIT_POSITION[1] # Right is positive x, forward is positive y
      theta0=euler
      if y_n_t>y0:
        fai = np.arctan((y_n_t-y0)/(x_n_t-x0))
        if fai<0:
          fai=fai+np.pi
      else:
        fai = np.arctan((y_n_t-y0)/(x_n_t-x0))
        if fai>0:
          fai=fai-np.pi
      alpha=fai-theta0
      if np.abs(alpha)>np.pi/2:
        f_1=f_2=1
      ld=((y_n_t-y0)**2+(x_n_t-x0)**2)**0.5
      dx=ld*np.cos(alpha)
      dy=ld*np.sin(alpha)
      dtheta=theta_n_t-theta0
      out_v=f_1*v_n_t+(1-f_1)*Kv*dx
      out_w=f_2*omega_n_t+(1-f_2)*Kw*(lambda1*dy+lambda2*dtheta)
      if out_v>max_v:
        out_v=max_v
      if out_w>max_w:
        out_w=max_w
      vel_msg.linear.x = out_v
      vel_msg.angular.z = omega[1]
      
    velocity_publisher.publish(vel_msg)
    
if __name__ == "__main__":
    pass