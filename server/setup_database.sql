/*
 * Postgresql: Setup Auvsi Imaging database
 * NOTE: this script assumes you're starting from 
 *       a fresh install of postgres, and may cause   
 *       cause errors other wise
 */

/* Step 1: create user `imaging_server` and database `auvsi` */
CREATE USER imaging_server WITH ENCRYPTED PASSWORD 'Byuauvs1';
CREATE DATABASE auvsi WITH OWNER imaging_server;
GRANT CONNECT ON DATABASE auvsi TO imaging_server;
GRANT USAGE ON SCHEMA public TO imaging_server;
GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO imaging_server;
GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO imaging_server;

/* Step 2: create all the tables */

CREATE TABLE "public"."incoming_image" (
  id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  nanoseconds int,
  image_path text NOT NULL,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."incoming_gps" (
  id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  nanoseconds int,
  latitude real NOT NULL,
  longitude real NOT NULL,
  altitude real NOT NULL,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."incoming_state" (
  id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  nanoseconds int,
  roll real NOT NULL,
  pitch real NOT NULL,
  yaw real NOT NULL,
  PRIMARY KEY ("id")
);


CREATE TABLE "public"."outgoing_data" (
  id serial NOT NULL,
  type text NOT NULL CHECK(type = "Standard" OR type = "off_axis" OR type = "emergent"),
  latitude real NOT NULL,
  longitude real NOT NULL,
  orientation text NOT NULL CHECK(orientation = "N" OR orientation = "NE" OR orientation = "E" OR orientation = "SE" OR orientation = "S" OR orientation = "SW" OR orientation = "W" OR orientation = "NW"),
  shape text NOT NULL CHECK(shape = "circle" OR shape = "semicircle" OR shape = "quarter_circle" OR shape = "triangle" OR shape = "square" OR shape = "rectangle" OR shape = "trapezoid" OR shape = "pentagon" OR shape = "hexagon" OR shape = "heptagon" OR shape = "octagon" OR shape = "star" OR shape = "cross"),
  background_color text NOT NULL CHECK(background_color = "white" OR background_color = "black" OR background_color = "gray" OR background_color = "red" OR background_color = "blue" OR background_color = "green" OR background_color = "yellow" OR background_color = "purple" OR background_color = "brown" OR background_color = "orange"),
  alphanumeric text NOT NULL,
  alphanumeric_color text NOT NULL CHECK(alphanumeric_color = "white" OR alphanumeric_color = "black" OR alphanumeric_color = "gray" OR alphanumeric_color = "red" OR alphanumeric_color = "blue" OR alphanumeric_color = "green" OR alphanumeric_color = "yellow" OR alphanumeric_color = "purple" OR alphanumeric_color = "brown" OR alphanumeric_color = "orange"),
  autonomous boolean NOT NULL,
  description text default '',
  cropped_image_path text NOT NULL,
  PRIMARY KEY("id")
);

/* If you want to destroy the auvsi_imaging user here's how you do it: */
-- DROP OWNED BY imaging_server;
-- DROP USER imaging_server;