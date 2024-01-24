import Head from "next/head";
import React from "react";
import Footer from "./Footer";
import Navigation from "../navigation/Navigation";

type Props = {
  children: React.ReactNode;
};

const Layout = ({ children }: Props) => {
  return (
    <>
      <Head>
        <title>Internarena</title>
        <meta name="description" content="Internarena" />
        <link rel="icon" href="/favicon.ico" />
      </Head>
      <div className="flex min-h-screen w-screen flex-col items-center">
        <Navigation />

        <main className="flex h-full w-screen flex-col items-center">
          {children}
        </main>
      </div>
      <Footer />
    </>
  );
};

export default Layout;
